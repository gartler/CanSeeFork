#ifndef _CANHANDLER_H_
#define _CANHANDLER_H_

#include <Arduino.h>

#include "config.h"

// Repository included libraries includes, see ./lib/ ************************
#include "ESP32CAN.h"
#include "CAN_config.h"

void can_init (CS_CONFIG_t *config);
void can_send (CAN_frame_t *frame, uint8_t bus);
void can_send_flow (uint16_t requestId);

#endif
