#ifndef _ISOTPHANDLER_H_
#define _ISOTPHANDLER_H_

#include <Arduino.h>

#include "config.h"
#include "canhandler.h"
#include "utils.h"
#include "zoe.h"

/**
 * @brief ISO-TP message.
 * @details see https://en.wikipedia.org/wiki/ISO_15765-2 for ISO-TP
 */
typedef struct
{
	uint32_t id = 0xffffffff;	  /**< @brief the from-address of the responding device */
	uint16_t length = 0;		  /**< @brief size, max 4096 */
	uint16_t index = 0;			  /**< @brief pointer */
	uint8_t next = 1;			  /**< @brief sequence of next frame */
	uint8_t data[4096];			  /**< @brief max ISOTP multiframe message */
	unsigned long flow_delay = 0; /**< @brief delay between outgoing isoMessageToString */
	uint8_t flow_counter = 0;	  /**< @brief frames to send (until new flow control) */
	uint8_t flow_active = 0;	  /**< @brief 0=no, 1=yes */
} ISO_MESSAGE_t;

void isotp_init();
void isotp_ticker();
void storeIsotpframe(CAN_frame_t *frame, uint8_t bus);
void can_send_flow(uint32_t requestId, uint8_t flow);
void requestIsotp(uint32_t id, int16_t length, uint8_t *request, uint8_t bus);
String isoMessageToString(ISO_MESSAGE_t *message);

#endif
