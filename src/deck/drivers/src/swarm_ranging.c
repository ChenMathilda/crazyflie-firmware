#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "system.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "assert.h"
#include "adhocdeck.h"
#include "ranging_struct.h"
#include "swarm_ranging.h"
#include "estimator_kalman.h"
#include "uart1_dma_task.h"
#define DEBUG_MODULE "SWARM"
#include "debug.h"

static uint16_t MY_UWB_ADDRESS;
/*--5添加--*/
static SemaphoreHandle_t rangingTableSetMutex;            // 用于互斥访问rangingTableSet
static median_data_t median_data[RANGING_TABLE_SIZE + 1]; // 存储测距的历史值
/*--5添加--*/
static QueueHandle_t rxQueue;
static Ranging_Table_Set_t rangingTableSet;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static int TfBufferIndex = 0;
static int rangingSeqNumber = 1;
/*--6添加--*/
static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
/*--6添加--*/
static float velocity;
static uint32_t generate_tick;
static uint32_t tickInterval;

// for AI///////////////////////
typedef struct
{
  float steer;
  float coll;
  float sign;
} aideck_control_t;
aideck_control_t aideck_control = {0.0f, 0.0f, 0.5f};
// for AI///////////////////////

int16_t distanceTowards[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = -1};

/*--4添加--*/
static neighborStateInfo_t neighborStateInfo; // 邻居的状态信息
static bool my_keep_flying;                   // 当前无人机的keep_flying
static uint8_t leader_address = 0;

void initNeighborStateInfoAndMedian_data()
{
  for (int i = 0; i < RANGING_TABLE_SIZE + 1; i++)
  {
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
  // for AI///////////////////////TODO Just need neighborAddress==0
  neighborStateInfo.steer = rangingMessageHeader->steer;
  neighborStateInfo.coll = rangingMessageHeader->coll;
  neighborStateInfo.sign = rangingMessageHeader->sign;

  // DEBUG_PRINT("setNeighborStateInfo:steer:%.2f,coll:%.2f,sign:%.2f\n", neighborStateInfo.steer, neighborStateInfo.coll, neighborStateInfo.sign);
  // for AI///////////////////////
  neighborStateInfo.refresh[neighborAddress] = true;
  if (neighborAddress == leader_address)
  { /* 无人机的keep_flying都是由0号无人机来设置的 */
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
bool get0AiStateInfo(float *steer, float *coll, float *sign)
{
  *steer = neighborStateInfo.steer;
  *coll = neighborStateInfo.coll;
  *sign = neighborStateInfo.sign;
  // DEBUG_PRINT("get0AiStateInfo:steer:%.2f,coll:%.2f,sign:%.2f\n", *steer, *coll, *sign);
  // memset(&neighborStateInfo.steer, 0, sizeof(float));
  // memset(*coll, 0, sizeof(float));
  // memset(&neighborStateInfo.sign, 0.5, sizeof(float));
  return true;
}
bool getOrSetKeepflying(uint16_t uwbAddress, bool keep_flying)
{
  if (uwbAddress == leader_address)
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
  /*--11添加--*/
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
  /*--11添加--*/
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
    /*--13添加--*/
    vTaskDelay(TX_PERIOD_IN_MS + rand() % 15);
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

    if (ABS(medianDist - calcDist) > 50)
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
  // for AI///////////////////////////just need when MY_UWB_ADDRESS==0
  if (MY_UWB_ADDRESS == 0)
  {
    if (uwbGetUartInfo(&aideck_control.steer, &aideck_control.coll, &aideck_control.sign)) // 获取到了AIdeck控制信息
    {
      rangingMessage->header.steer = aideck_control.steer;
      rangingMessage->header.coll = aideck_control.coll;
      rangingMessage->header.sign = aideck_control.sign;
    }
    // else // 没有获取到AIdeck控制信息
    // {
    //   rangingMessage->header.steer = 0.0;
    //   // rangingMessage->header.coll = aideck_control.coll;
    //   rangingMessage->header.sign = 0.5;
    // }
    // DEBUG_PRINT("generate0:steer=%.2f, coll=%.2f, sign=%.2f\n", rangingMessage->header.steer, rangingMessage->header.coll, rangingMessage->header.sign);
  }
  else if (MY_UWB_ADDRESS != 0)
  {
    rangingMessage->header.steer = neighborStateInfo.steer;
    rangingMessage->header.coll = neighborStateInfo.coll;
    rangingMessage->header.sign = neighborStateInfo.sign;
    // DEBUG_PRINT("generate!0:steer=%.2f, coll=%.2f, sign=%.2f\n", rangingMessage->header.steer, rangingMessage->header.coll, rangingMessage->header.sign);
  }
  // for AI///////////////////////////
  rangingMessage->header.keep_flying = my_keep_flying;
  /*--9添加--*/
  return rangingMessage->header.msgLength;
}

LOG_GROUP_START(Ranging)
LOG_ADD(LOG_INT16, distTo0, distanceTowards + 0)
LOG_ADD(LOG_INT16, distTo1, distanceTowards + 1)
LOG_ADD(LOG_INT16, distTo2, distanceTowards + 2)
LOG_ADD(LOG_INT16, distTo3, distanceTowards + 3)
LOG_ADD(LOG_INT16, distTo4, distanceTowards + 4)
LOG_ADD(LOG_INT16, distTo5, distanceTowards + 5)
LOG_ADD(LOG_INT16, distTo6, distanceTowards + 6)
LOG_ADD(LOG_INT16, distTo7, distanceTowards + 7)
LOG_ADD(LOG_INT16, distTo8, distanceTowards + 8)
LOG_ADD(LOG_INT16, distTo8, distanceTowards + 9)
LOG_GROUP_STOP(Ranging)

LOG_GROUP_START(AIdeck_control)
LOG_ADD(LOG_FLOAT, Steer, &aideck_control.steer)
LOG_ADD(LOG_FLOAT, Coll, &aideck_control.coll)
LOG_ADD(LOG_FLOAT, Sign, &aideck_control.sign)
LOG_GROUP_STOP(AIdeck_control)
