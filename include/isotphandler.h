#ifndef _ISOTPHANDLER_H_
#define _ISOTPHANDLER_H_

#include <Arduino.h>

#include "config.h"
#include "canhandler.h"
#include "utils.h"
#include "zoe.h"

#define DEBUG_COMMAND         0x04
#define DEBUG_BUS_RECEIVE_ISO 0x02

// ISO-TP message ************************************************************
typedef struct {
  uint32_t id = 0xffffffff;                        // the from-address of the responding device
  uint32_t requestId = 0xffffffff;                 // the to-address of the device to send the request to
  uint16_t length = 0;
  uint8_t index = 0;
  uint8_t next = 1;
  uint8_t request[8];
  uint8_t requestLength = 0;
  // uint8_t reply[8];
  // uint8_t replyLength = 0;
  // uint8_t* data = 0;
  uint8_t data[1024];
} ISO_MESSAGE_t;

void isotp_init (CS_CONFIG_t *config, void (*p)(String o));
void storeIsotpframe (CAN_frame_t &frame, uint8_t bus);
String isoMessageToString(ISO_MESSAGE_t &message);
void can_send_flow (uint16_t requestId, uint8_t flow);
String isoMessageToString(ISO_MESSAGE_t &message);

#endif
