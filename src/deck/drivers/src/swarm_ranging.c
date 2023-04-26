#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "time.h"
#include "math.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "assert.h"
#include "adhocdeck.h"
#include "ranging_struct.h"
#include "swarm_ranging.h"
#include "estimator_kalman.h"

static uint16_t MY_UWB_ADDRESS;
/*--5添加--*/
static SemaphoreHandle_t rangingTableSetMutex;                 // 用于互斥访问rangingTableSet
static median_data_t median_data[RANGING_TABLE_SIZE + 1];      // 存储测距的历史值
static uint16_t rv_data_interval[RANGING_TABLE_SIZE + 1];      // 两次接收到数据包的时间间隔
static uint8_t rv_data_interval_index[RANGING_TABLE_SIZE + 1]; // 两次接收到数据包的时间间隔下标
static uint8_t rv_any_index = 0;
static uint8_t temp;
static currentNeighborAddressInfo_t currentNeighborAddressInfo;
static uint32_t latest_txTime;                                                  // 最新的发送数据包时间，用于日志
static uint32_t neighbor_latest_rvTime[RANGING_TABLE_SIZE + 1];                 // 最新的接收数据包时间，用于日志
static uint32_t last_swapPeriod_Time;                                           // 上一次变化周期的时间，如果距离上一次变换周期的时间>固定的传输周期，则恢复至固定传输周期
static uint32_t last_swapPeriod_period;                                         // 上一次变化的周期值
static tx_rv_interval_history_t tx_rv_interval_history[RANGING_TABLE_SIZE + 1]; //  两次的漂移差
static uint8_t tx_rv_interval = 0;
static uint8_t nextTransportPeriod = TX_PERIOD_IN_MS; // 发送数据包周期

/*--5添加--*/
static QueueHandle_t rxQueue;
static Ranging_Table_Set_t rangingTableSet;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static int TfBufferIndex = 0;
static uint8_t rangingSeqNumber = 1;
static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static float velocity;

int16_t distanceTowards[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = -1};

/*--4添加--*/
static neighborStateInfo_t neighborStateInfo; // 邻居的状态信息
static bool my_keep_flying;                   // 当前无人机的keep_flying

void initNeighborStateInfoAndMedian_data()
{
  for (int i = 0; i < RANGING_TABLE_SIZE + 1; i++)
  {
    tx_rv_interval_history[i].latest_data_index = 0;
    tx_rv_interval_history[i].interval[0] = 1000;
    median_data[i].index_inserting = 0;
    neighborStateInfo.refresh[i] = false;
  }
  my_keep_flying = false;
}

void setNeighborStateInfo(uint16_t neighborAddress, int16_t distance, Ranging_Message_Header_t *rangingMessageHeader)
{
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  neighborStateInfo.distanceTowards[neighborAddress] = distance;
  neighborStateInfo.velocityXInWorld[neighborAddress] = rangingMessageHeader->velocityXInWorld;
  neighborStateInfo.velocityYInWorld[neighborAddress] = rangingMessageHeader->velocityYInWorld;
  neighborStateInfo.gyroZ[neighborAddress] = rangingMessageHeader->gyroZ;
  neighborStateInfo.positionZ[neighborAddress] = rangingMessageHeader->positionZ;
  neighborStateInfo.refresh[neighborAddress] = true;
  if (neighborAddress == 1)
  { /*无人机的keep_flying都是由0号无人机来设置的*/
    my_keep_flying = rangingMessageHeader->keep_flying;
  }
}
void setNeighborStateInfo_isNewAdd(uint16_t neighborAddress, bool isNewAddNeighbor)
{
  // DEBUG_PRINT("--%d,%d,%d\n", isNewAddNeighbor, neighborStateInfo.isNewAddUsed[neighborAddress], neighborAddress);
  if (isNewAddNeighbor == true)
  {
    neighborStateInfo.isNewAdd[neighborAddress] = true;
    neighborStateInfo.isNewAddUsed[neighborAddress] = false;
  }
  else
  {
    if (neighborStateInfo.isNewAddUsed[neighborAddress] == true)
    {
      neighborStateInfo.isNewAdd[neighborAddress] = false;
    }
  }
}

bool getNeighborStateInfo(uint16_t neighborAddress, uint16_t *distance, short *vx, short *vy, float *gyroZ, uint16_t *height, bool *isNewAddNeighbor)
{
  if (neighborStateInfo.refresh[neighborAddress] == true && my_keep_flying == true)
  {
    neighborStateInfo.refresh[neighborAddress] = false;
    *distance = neighborStateInfo.distanceTowards[neighborAddress];
    *vx = neighborStateInfo.velocityXInWorld[neighborAddress];
    *vy = neighborStateInfo.velocityYInWorld[neighborAddress];
    *gyroZ = neighborStateInfo.gyroZ[neighborAddress];
    *height = neighborStateInfo.positionZ[neighborAddress];
    *isNewAddNeighbor = neighborStateInfo.isNewAdd[neighborAddress];
    neighborStateInfo.isNewAddUsed[neighborAddress] = true;
    return true;
  }
  else
  {
    return false;
  }
}

bool getOrSetKeepflying(uint16_t uwbAddress, bool keep_flying)
{
  if (uwbAddress == 1)
  {
    my_keep_flying = keep_flying;
    return keep_flying;
  }
  else
  {
    return my_keep_flying;
  }
}

void getCurrentNeighborAddressInfo_t(currentNeighborAddressInfo_t *currentNeighborAddressInfo)
{
  xSemaphoreTake(rangingTableSetMutex, portMAX_DELAY);
  currentNeighborAddressInfo->size = 0;
  int i = 0;
  set_index_t iter = rangingTableSet.fullQueueEntry;
  while (iter != -1)
  {
    Ranging_Table_Set_Item_t cur = rangingTableSet.setData[iter];
    currentNeighborAddressInfo->address[i] = cur.data.neighborAddress;
    currentNeighborAddressInfo->size++;
    i++;
    iter = cur.next;
  }
  xSemaphoreGive(rangingTableSetMutex);
}

uint16_t get_tx_rx_min_interval(address_t address)
{
  uint8_t index = tx_rv_interval_history[address].latest_data_index;
  uint16_t res = tx_rv_interval_history[address].interval[index];

  for (uint8_t i = 0; i < TX_RV_INTERVAL_HISTORY_SIZE - 1; i++)
  {
    index = (index + 1) % TX_RV_INTERVAL_HISTORY_SIZE;
    uint16_t interval = tx_rv_interval_history[address].interval[index];
    res = res < interval ? res : interval;
  }
  return res;
}

static int16_t median_filter_3(int16_t *data)
{
  int16_t middle;
  if ((data[0] <= data[1]) && (data[0] <= data[2]))
  {
    middle = (data[1] <= data[2]) ? data[1] : data[2];
  }
  else if ((data[1] <= data[0]) && (data[1] <= data[2]))
  {
    middle = (data[0] <= data[2]) ? data[0] : data[2];
  }
  else
  {
    middle = (data[0] <= data[1]) ? data[0] : data[1];
  }
  return middle;
}
#define ABS(a) ((a) > 0 ? (a) : -(a))

/*--4添加--*/

void rangingRxCallback(void *parameters)
{
  // DEBUG_PRINT("rangingRxCallback \n");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  UWB_Packet_t *packet = (UWB_Packet_t *)parameters;

  dwTime_t rxTime;
  dwt_readrxtimestamp((uint8_t *)&rxTime.raw);
  Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
  rxMessageWithTimestamp.rxTime = rxTime;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *)packet->payload;
  rxMessageWithTimestamp.rangingMessage = *rangingMessage;

  xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingTxCallback(void *parameters)
{
  dwTime_t txTime;
  dwt_readtxtimestamp((uint8_t *)&txTime.raw);
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex].seqNumber = rangingSeqNumber;
  TfBuffer[TfBufferIndex].timestamp = txTime;
}

int16_t getDistance(uint16_t neighborAddress)
{
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  return distanceTowards[neighborAddress];
}

void setDistance(uint16_t neighborAddress, int16_t distance)
{
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  distanceTowards[neighborAddress] = distance;
}

static void uwbRangingTxTask(void *parameters)
{
  systemWaitStart();

  /* velocity log variable id */
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = RANGING;
  //  txPacketCache.header.mac = ? TODO init mac header
  while (true)
  {
    int msgLen = generateRangingMessage((Ranging_Message_t *)&txPacketCache.payload);
    txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
    uwbSendPacketBlock(&txPacketCache);
    uint8_t last_period = xTaskGetTickCount() - latest_txTime;
    latest_txTime = xTaskGetTickCount();
    /*--13添加--*/
    getCurrentNeighborAddressInfo_t(&currentNeighborAddressInfo);
    uint16_t notget_packet_interval = 0;

    nextTransportPeriod = TX_PERIOD_IN_MS;

    for (int i = 0; i < currentNeighborAddressInfo.size; i++)
    {
      address_t address = currentNeighborAddressInfo.address[i];
      notget_packet_interval = xTaskGetTickCount() - neighbor_latest_rvTime[address];
      if (notget_packet_interval > 45)
      {
        // if (get_tx_rx_min_interval(address) <= 2 /*|| get_tx_rx_min_interval(address) >= 18*/)
        // {
        //   nextTransportPeriod = TX_PERIOD_IN_MS / 4 + rand() % 25;
        //   break;
        // }
        nextTransportPeriod = TX_PERIOD_IN_MS / 4 + rand() % 25;
        break;
      }
    }
    // nextTransportPeriod = 15 + rand() % 6;
    vTaskDelay(nextTransportPeriod);

    /*--13添加--*/
  }
}

static void uwbRangingRxTask(void *parameters)
{
  systemWaitStart();

  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true)
  {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY))
    {
      //  DEBUG_PRINT("uwbRangingRxTask: received ranging message \n");
      processRangingMessage(&rxPacketCache);
    }
  }
}

void rangingInit()
{
  MY_UWB_ADDRESS = getUWBAddress();
  DEBUG_PRINT("MY_UWB_ADDRESS = %d \n", MY_UWB_ADDRESS);
  /*--12添加--*/
  initNeighborStateInfoAndMedian_data();
  rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
  rangingTableSetMutex = xSemaphoreCreateMutex();
  srand((unsigned)time(NULL));
  /*--12添加--*/
  rangingTableSetInit(&rangingTableSet);

  listener.type = RANGING;
  listener.rxQueue = NULL; // handle rxQueue in swarm_ranging.c instead of adhocdeck.c
  listener.rxCb = rangingRxCallback;
  listener.txCb = rangingTxCallback;
  uwbRegisterListener(&listener);

  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle); // TODO optimize STACK SIZE
}

int16_t computeDistance(uint16_t neighborAddress, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf)
{
  // DEBUG_PRINT("compute start");

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, tprop_ctn;
  tRound1 = (Rr.timestamp.full - Tp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply1 = (Tr.timestamp.full - Rp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tRound2 = (Rf.timestamp.full - Tr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply2 = (Tf.timestamp.full - Rr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  tprop_ctn = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);
  int16_t calcDist = (int16_t)tprop_ctn * 0.4691763978616;
  /*--7添加--*/
  /*这里暂时采用和李树帅twr中一样的形式*/

  if (calcDist > 0 && calcDist < 1000)
  {

    int16_t medianDist = median_filter_3(median_data[neighborAddress].distance_history);

    median_data[neighborAddress].index_inserting++;
    if (median_data[neighborAddress].index_inserting == 3)
    {
      median_data[neighborAddress].index_inserting = 0;
    }
    median_data[neighborAddress].distance_history[median_data[neighborAddress].index_inserting] = calcDist;

    if (ABS(medianDist - calcDist) > 15)
    {
      return medianDist;
    }
    else
    {
      return calcDist;
    }
  }
  return 0;
  /*--7添加--*/

  // bool isErrorOccurred = false;
  // if (calcDist > 1000 || calcDist < 0)
  // {
  //   // DEBUG_PRINT("isErrorOccurred\n");
  //   isErrorOccurred = true;
  // }

  // if (tRound2 < 0 || tReply2 < 0)
  // {
  //   DEBUG_PRINT("tRound2 < 0 || tReply2 < 0\n");
  //   isErrorOccurred = true;
  // }

  // if (isErrorOccurred)
  // {
  //   return 0;
  // }

  // return distance;
}

void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp)
{
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t neighborAddress = rangingMessage->header.srcAddress;
  set_index_t neighborIndex = findInRangingTableSet(&rangingTableSet, neighborAddress);

  /*这里用于测试数据丢包情况*/
  rv_data_interval[neighborAddress] = xTaskGetTickCount() - neighbor_latest_rvTime[neighborAddress];
  rv_data_interval_index[neighborAddress] = (rv_data_interval_index[neighborAddress] + 1) % 10;
  rv_any_index++;
  /*这里用于测试数据丢包情况*/
  uint32_t curr_time = xTaskGetTickCount();
  neighbor_latest_rvTime[neighborAddress] = curr_time;
  tx_rv_interval_history[neighborAddress].latest_data_index = (tx_rv_interval_history[neighborAddress].latest_data_index + 1) % TX_RV_INTERVAL_HISTORY_SIZE;
  tx_rv_interval_history[neighborAddress].interval[tx_rv_interval_history[neighborAddress].latest_data_index] = curr_time - latest_txTime;

  if (neighborAddress == 5)
  {
    tx_rv_interval = curr_time - latest_txTime;
  }
  // DEBUG_PRINT("%d,%d,%d\n", neighborAddress, tx_rv_interval_history[neighborAddress].latest_data_index, tx_rv_interval_history[neighborAddress].interval[tx_rv_interval_history[neighborAddress].latest_data_index]);
  // if (tx_rv_interval <= 1)
  // {

  //   nextTransportPeriod = TX_PERIOD_IN_MS - 5 + rand() % 10;
  //   last_swapPeriod_Time = xTaskGetTickCount();
  //   last_swapPeriod_period = nextTransportPeriod;
  // }
  // else
  // {
  //   if ((xTaskGetTickCount() - last_swapPeriod_Time) > nextTransportPeriod + 3)
  //   {
  //     nextTransportPeriod = TX_PERIOD_IN_MS;
  //   }
  //   // DEBUG_PRINT("..%d\n", nextTransportPeriod);
  // }

  /*--8添加--*/
  bool isNewAddNeighbor = neighborIndex == -1 ? true : false; /*如果是新添加的邻居，则是true*/
  setNeighborStateInfo_isNewAdd(neighborAddress, isNewAddNeighbor);
  /*--8添加--*/

  // DEBUG_PRINT("processRangingMessage:%d\n", isNewAddNeighbor);
  /* handle new neighbor */
  if (neighborIndex == -1)
  {
    if (rangingTableSet.freeQueueEntry == -1)
    {
      /* ranging table set is full, ignore this ranging message */
      return;
    }
    Ranging_Table_t table;
    rangingTableInit(&table, neighborAddress);
    /*--10添加--*/
    xSemaphoreTake(rangingTableSetMutex, portMAX_DELAY);
    neighborIndex = rangingTableSetInsert(&rangingTableSet, &table);
    xSemaphoreGive(rangingTableSetMutex);
    /*--10添加--*/
  }

  Ranging_Table_t *neighborRangingTable = &rangingTableSet.setData[neighborIndex].data;
  Ranging_Table_Tr_Rr_Buffer_t *neighborTrRrBuffer = &neighborRangingTable->TrRrBuffer;

  /* update Re */
  neighborRangingTable->Re.timestamp = rangingMessageWithTimestamp->rxTime;
  neighborRangingTable->Re.seqNumber = rangingMessage->header.msgSequence;

  /* update Tr and Rr */
  Timestamp_Tuple_t neighborTr = rangingMessage->header.lastTxTimestamp;
  if (neighborTr.timestamp.full && neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.timestamp.full && neighborTr.seqNumber == neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.seqNumber)
  {
    rangingTableBufferUpdate(&neighborRangingTable->TrRrBuffer,
                             neighborTr,
                             neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr);
  }

  /* update Rf */ /*只有rf需要地址？？？*/
  Timestamp_Tuple_t neighborRf = {.timestamp.full = 0};
  if (rangingMessage->header.filter & (1 << (getUWBAddress() % 16)))
  {
    /* retrieve body unit */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++)
    {
      if (rangingMessage->bodyUnits[i].address == getUWBAddress())
      {
        neighborRf = rangingMessage->bodyUnits[i].timestamp;
        break;
      }
    }
  }

  if (neighborRf.timestamp.full)
  {

    neighborRangingTable->Rf = neighborRf;
    // TODO it is possible that can not find corresponding Tf
    /* find corresponding Tf in TfBuffer */
    for (int i = 0; i < Tf_BUFFER_POOL_SIZE; i++)
    {
      if (TfBuffer[i].seqNumber == neighborRf.seqNumber)
      {
        neighborRangingTable->Tf = TfBuffer[i];
      }
    }

    Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetCandidate(&neighborRangingTable->TrRrBuffer,
                                                                                     neighborRangingTable->Tf);
    /* try to compute distance */
    if (Tr_Rr_Candidate.Tr.timestamp.full && Tr_Rr_Candidate.Rr.timestamp.full &&
        neighborRangingTable->Tp.timestamp.full && neighborRangingTable->Rp.timestamp.full &&
        neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full)
    {

      int16_t distance = computeDistance(neighborAddress, neighborRangingTable->Tp, neighborRangingTable->Rp,
                                         Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,
                                         neighborRangingTable->Tf, neighborRangingTable->Rf);
      if (distance != 0)
      {
        neighborRangingTable->distance = distance;
        setDistance(neighborRangingTable->neighborAddress, distance);
        /*--9添加--*/
        setNeighborStateInfo(neighborAddress, distance, &rangingMessage->header);
        /*--9添加--*/
      }
      else
      {
        // DEBUG_PRINT("distance is not updated since some error occurs\n");
      }
    }
  }

  /* Tp <- Tf, Rp <- Rf */
  if (neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full)
  {
    rangingTableShift(neighborRangingTable);
  }

  /* update Rr */
  neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr = neighborRangingTable->Re;

  /* update expiration time */
  neighborRangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);

  neighborRangingTable->state = RECEIVED;
}

int generateRangingMessage(Ranging_Message_t *rangingMessage)
{
  /*--9添加--*/
  xSemaphoreTake(rangingTableSetMutex, portMAX_DELAY);
#ifdef ENABLE_BUS_BOARDING_SCHEME
  sortRangingTableSet(&rangingTableSet);
#endif
  rangingTableSetClearExpire(&rangingTableSet);
  xSemaphoreGive(rangingTableSetMutex);
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++; // 每产生一次消息，序列号+1
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0; // filter设置为0
  /* generate message body */
  for (set_index_t index = rangingTableSet.fullQueueEntry; index != -1;
       index = rangingTableSet.setData[index].next)
  {
    Ranging_Table_t *table = &rangingTableSet.setData[index].data;
    if (bodyUnitNumber >= MAX_BODY_UNIT_NUMBER)
    {
      break;
    }
    if (table->state == RECEIVED)
    {
      rangingMessage->bodyUnits[bodyUnitNumber].address = table->neighborAddress;
      /* It is possible that Re is not the newest timestamp, because the newest may be in rxQueue
       * waiting to be handled.
       */
      rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->Re; // 为什么是re???
      bodyUnitNumber++;
      table->state = TRANSMITTED;
      rangingMessage->header.filter |= 1 << (table->neighborAddress % 16); // 应该是用来识别地址的吧
    }
  }
  /* generate message header */
  rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
  rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
  rangingMessage->header.msgSequence = curSeqNumber;
  rangingMessage->header.lastTxTimestamp = TfBuffer[TfBufferIndex];
  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);
  velocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2) + pow(velocityZ, 2));
  /* velocity in cm/s */
  rangingMessage->header.velocity = (short)(velocity * 100);

  estimatorKalmanGetSwarmInfo(&rangingMessage->header.velocityXInWorld, &rangingMessage->header.velocityYInWorld, &rangingMessage->header.gyroZ, &rangingMessage->header.positionZ);
  rangingMessage->header.keep_flying = my_keep_flying;

  /*--9添加--*/
  return rangingMessage->header.msgLength;
}

LOG_GROUP_START(Ranging)
LOG_ADD(LOG_INT16, distTo1, distanceTowards + 1)
LOG_ADD(LOG_INT16, distTo2, distanceTowards + 2)
LOG_ADD(LOG_INT16, distTo3, distanceTowards + 3)
LOG_ADD(LOG_INT16, distTo4, distanceTowards + 4)
LOG_ADD(LOG_INT16, distTo5, distanceTowards + 5)
LOG_ADD(LOG_INT16, distTo6, distanceTowards + 6)
LOG_ADD(LOG_INT16, distTo7, distanceTowards + 7)
LOG_ADD(LOG_INT16, distTo8, distanceTowards + 8)
LOG_ADD(LOG_INT16, distTo8, distanceTowards + 9)

LOG_ADD(LOG_UINT8, index, &rv_any_index)               // 只要接收到数据包，index++
LOG_ADD(LOG_UINT8, index0, rv_data_interval_index + 0) // 接收到0号无人机数据包，rv_data_interval_index[0]++
LOG_ADD(LOG_UINT8, index1, rv_data_interval_index + 1)
LOG_ADD(LOG_UINT8, index2, rv_data_interval_index + 2)
LOG_ADD(LOG_UINT8, index3, rv_data_interval_index + 3)
LOG_ADD(LOG_UINT8, index4, rv_data_interval_index + 4)
LOG_ADD(LOG_UINT8, index5, rv_data_interval_index + 5)
LOG_ADD(LOG_UINT8, index6, rv_data_interval_index + 6)

LOG_ADD(LOG_UINT8, diff5, &tx_rv_interval) // 与5号无人机的漂移差

LOG_ADD(LOG_UINT16, interval0, rv_data_interval + 0) // 连续两次接收到0号无人机的时间差
LOG_ADD(LOG_UINT16, interval1, rv_data_interval + 1)
LOG_ADD(LOG_UINT16, interval2, rv_data_interval + 2)
LOG_ADD(LOG_UINT16, interval3, rv_data_interval + 3)
LOG_ADD(LOG_UINT16, interval4, rv_data_interval + 4)
LOG_ADD(LOG_UINT16, interval5, rv_data_interval + 5)
LOG_ADD(LOG_UINT16, interval6, rv_data_interval + 6)
LOG_ADD(LOG_UINT8, period, &nextTransportPeriod)
LOG_ADD(LOG_UINT8, seq, &rangingSeqNumber)
LOG_GROUP_STOP(Ranging)
