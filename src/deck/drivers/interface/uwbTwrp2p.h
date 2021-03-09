/*
 * uwbTwrp2p.h
 *
 *  Created on: Dec 2, 2020
 *      Author: bitcraze
 */

#ifndef SRC_DECK_DRIVERS_INTERFACE_UWBTWRP2P_H_
#define SRC_DECK_DRIVERS_INTERFACE_UWBTWRP2P_H_
#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

#define LPS_TWRP2P_POLL 0x01   // Poll is initiated by the tag
#define LPS_TWRP2P_ANSWER 0x02
#define LPS_TWRP2P_FINAL 0x03
#define LPS_TWRP2P_REPORT 0x04 // Report contains all measurement from the anchor

#define LPS_TWRP2P_TYPE 0
#define LPS_TWRP2P_SEQ 1

extern uwbAlgorithm_t uwbTWRP2PAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) lpsTWRp2pReportPayload_t;

#define MAX_UWB_RECEIVE_TIMEOUT 65 //65ms is max interval


#endif /* SRC_DECK_DRIVERS_INTERFACE_UWBTWRP2P_H_ */
