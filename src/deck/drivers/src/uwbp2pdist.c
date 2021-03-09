/*
 * uwbp2pdist.c
 *
 *  Created on: Dec 2, 2020
 *      Author: bitcraze
 */

#define DEBUG_MODULE "DWM"

#include <string.h>

#include "uwbp2pdist.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "log.h"
#include "crtp_localization_service.h"

#include "physicalConstants.h"

#define ANTENNA_OFFSET 154.6   // In meter

static int ANTENNA_DELAY = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static volatile uint8_t curr_seq = 0;
static int curr_peer = 0;

float pressure = 0;
float temperature = 0;
float asl = 0;
bool pressure_ok = true;

static float pdistance = 0;
static uint32_t timeout_p2p=0;
static uint32_t default_twr_interval=8000;

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (ANTENNA_DELAY / 2);

  switch (txPacket.payload[0]) {
    case LPS_P2P_POLL:
      DEBUG_PRINT("sent LPS_P2P_POLL\n");
      poll_tx = departure;
      break;
    case LPS_P2P_FINAL:
      DEBUG_PRINT("sent LPS_P2P_FINAL\n");
      final_tx = departure;
      break;
    case LPS_P2P_ANSWER:
      DEBUG_PRINT("sent LPS_P2P_ANSWER to %02x at %04x\n", (unsigned int)txPacket.destAddress, (unsigned int)departure.low32);
      answer_tx = departure;
      break;
    case LPS_P2P_REPORT:
      DEBUG_PRINT("sent LPS_P2P_REPORT\n");
      break;
  }
}

static uint32_t rxcallback(dwDevice_t *dev)
{
  dwTime_t arival = { .full=0 };
  dwGetReceiveTimestamp(dev, &arival);

  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) return 0;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  //DEBUG_PRINT("txPacket.destAddress = %d\n", (unsigned int)txPacket.destAddress);
  DEBUG_PRINT("rxPacket.payload[LPS_P2P_TYPE]) = %d\n", (unsigned int)rxPacket.payload[LPS_P2P_TYPE]);
  switch(rxPacket.payload[LPS_P2P_TYPE]) {
    case LPS_P2P_POLL:
    {
    	DEBUG_PRINT("LPS_P2P_POLL\n");

    	curr_peer = rxPacket.sourceAddress;

    	int payloadLength = 2;
    	txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_ANSWER;
    	txPacket.payload[LPS_P2P_SEQ] = rxPacket.payload[LPS_P2P_SEQ];

    	arival.full -= (ANTENNA_DELAY/2);
    	poll_rx = arival;

    	dwNewTransmit(dev);
    	dwSetDefaults(dev);
    	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+payloadLength);
    	dwWaitForResponse(dev, true);
    	dwStartTransmit(dev);
    	break;
    }
    // Tag received messages
    case LPS_P2P_ANSWER:
    	DEBUG_PRINT("LPS_P2P_ANSWER\n");
    	if (rxPacket.payload[LPS_P2P_SEQ] != curr_seq) return 0;

    	txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_FINAL;
    	txPacket.payload[LPS_P2P_SEQ] = rxPacket.payload[LPS_P2P_SEQ];

    	arival.full -= (ANTENNA_DELAY / 2);
    	answer_rx = arival;

    	dwNewTransmit(dev);
    	dwSetDefaults(dev);
    	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
    	dwWaitForResponse(dev, true);
    	dwStartTransmit(dev);
    	break;
    case LPS_P2P_FINAL:
    	DEBUG_PRINT("LPS_P2P_FINAL\n");
      //if (curr_peer != rxPacket.sourceAddress) return 0;

    	lpsp2pTagReportPayload_t *report = (lpsp2pTagReportPayload_t *)(txPacket.payload+2);

    	arival.full -= (ANTENNA_DELAY/2);
    	final_rx = arival;

    	txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_REPORT;
    	txPacket.payload[LPS_P2P_SEQ] = rxPacket.payload[LPS_P2P_SEQ];
    	memcpy(&report->pollRx, &poll_rx, 5);
    	memcpy(&report->answerTx, &answer_tx, 5);
    	memcpy(&report->finalRx, &final_rx, 5);
    	report->pressure = pressure;
    	report->temperature = temperature;
    	report->asl = asl;
    	report->pressure_ok = pressure_ok;

    	dwNewTransmit(dev);
    	dwSetDefaults(dev);
    	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsp2pTagReportPayload_t));
    	dwWaitForResponse(dev, true);
    	dwStartTransmit(dev);
    	timeout_p2p = default_twr_interval/2; //set a shorter delay to sent next poll
    	break;
    case LPS_P2P_REPORT:
    {
    	DEBUG_PRINT("LPS_P2P_REPORT\n");
    	lpsp2pTagReportPayload_t *report = (lpsp2pTagReportPayload_t *)(rxPacket.payload+2);
    	double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

    	if (rxPacket.payload[LPS_P2P_SEQ] != curr_seq) {
    		return 0;
    	}

    	memcpy(&poll_rx, &report->pollRx, 5);
    	memcpy(&answer_tx, &report->answerTx, 5);
    	memcpy(&final_rx, &report->finalRx, 5);

    	tround1 = answer_rx.low32 - poll_tx.low32;
    	treply1 = answer_tx.low32 - poll_rx.low32;
    	tround2 = final_rx.low32 - answer_tx.low32;
    	treply2 = final_tx.low32 - answer_rx.low32;

    	tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

    	tprop = tprop_ctn / LOCODECK_TS_FREQ;
    	pdistance = SPEED_OF_LIGHT * tprop;

    	DEBUG_PRINT("Distance = %d\n",(int)(100*pdistance));
    	dwNewReceive(dev);
    	dwSetDefaults(dev);
    	dwStartReceive(dev);
    	break;
    }
    default:
    {
    	DEBUG_PRINT("rxcallback End!\n\n");
    	dwNewTransmit(dev);
    	dwSetDefaults(dev);
    	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
    	dwWaitForResponse(dev, true);
    	dwStartTransmit(dev);
    	break;
    }

  }
  DEBUG_PRINT("rxcallback Finished!\n\n");
  return timeout_p2p;
}

static void initiateRanging(dwDevice_t *dev)
{
  dwIdle(dev);

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  curr_seq = 0;
  curr_peer = 0;

  pressure = temperature = asl = 0;
  pressure_ok = true;

  txPacket.sourceAddress = 0xbccf000000000000 | 12;
  txPacket.destAddress = 0xbccf000000000000 | 13;
  txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_POLL;
  txPacket.payload[LPS_P2P_SEQ] = ++curr_seq;

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  DEBUG_PRINT("Crazyflie to Crazyflie!\n");
  DEBUG_PRINT("txPacket.destAddress = %d\n", (unsigned int)txPacket.destAddress);
}

static uint32_t p2pDistOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
      //DEBUG_PRINT("eventPacketReceived\n"),
      timeout_p2p=default_twr_interval;
      rxcallback(dev);
      break;
    case eventPacketSent:
      //DEBUG_PRINT("eventPacketSent\n"),
      txcallback(dev);
      break;
    case eventReceiveTimeout:
      //DEBUG_PRINT("eventReceiveTimeout\n"),
      dwNewReceive(dev);
      dwSetDefaults(dev);
      dwStartReceive(dev);
      timeout_p2p = (timeout_p2p>MAX_UWB_RECEIVE_TIMEOUT ? timeout_p2p-MAX_UWB_RECEIVE_TIMEOUT : 0) ;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      //DEBUG_PRINT("eventTimeout\n"),
      initiateRanging(dev);
      timeout_p2p=default_twr_interval;
      break;
    case eventReceiveFailed:
      return 0;
  }

  return timeout_p2p;
}

static void p2pDistInit(dwDevice_t *dev)
{
  return;
  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  dwSetReceiveWaitTimeout(dev, 0);
  dwCommitConfiguration(dev);
}

static bool isRangingOk()
{
  return true;
}

point_t anchorPosition[2];
static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
    *position = anchorPosition[anchorId];
    return true;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  return 2;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  return 2;
}
uwbAlgorithm_t uwbP2PDistAlgorithm = {
  .init = p2pDistInit,
  .onEvent = p2pDistOnEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

//LOG_GROUP_START(peerdist)
//LOG_ADD(LOG_FLOAT, distance2peer, &pdistance)
//LOG_GROUP_STOP(peerdist)


