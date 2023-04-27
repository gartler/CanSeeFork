#ifndef _CANHANDLER_H_
#define _CANHANDLER_H_

#include <Arduino.h>

// Repository included libraries includes, see ./lib/ ************************
#include <driver/twai.h>
#include <driver/gpio.h>

#include "config.h"
#include "serialhandler.h"
#include "utils.h"

// for historical reasons, we use the can_frame definition of Thomas Barth and Michael Wagner

/** \brief CAN Node Bus speed */
typedef enum {
	CAN_SPEED_100KBPS = 100,  /**< \brief CAN Node runs at 100kBit/s. */
	CAN_SPEED_125KBPS = 125,  /**< \brief CAN Node runs at 125kBit/s. */
	CAN_SPEED_200KBPS = 200,  /**< \brief CAN Node runs at 250kBit/s. */
	CAN_SPEED_250KBPS = 250,  /**< \brief CAN Node runs at 250kBit/s. */
	CAN_SPEED_500KBPS = 500,  /**< \brief CAN Node runs at 500kBit/s. */
	CAN_SPEED_800KBPS = 800,  /**< \brief CAN Node runs at 800kBit/s. */
	CAN_SPEED_1000KBPS = 1000 /**< \brief CAN Node runs at 1000kBit/s. */
} CAN_speed_t;

/**
 * \brief CAN frame type (standard/extended)
 */
typedef enum {
	CAN_frame_std = 0, /**< Standard frame, using 11 bit identifer. */
	CAN_frame_ext = 1  /**< Extended frame, using 29 bit identifer. */
} CAN_frame_format_t;

/**
 * \brief CAN RTR
 */
typedef enum {
	CAN_no_RTR = 0, /**< No RTR frame. */
	CAN_RTR = 1     /**< RTR frame. */
} CAN_RTR_t;

/** \brief Frame information record type */
typedef union {
	uint32_t U; /**< \brief Unsigned access */
	struct {
		uint8_t DLC : 4;               /**< \brief [3:0] DLC, Data length container */
		unsigned int unknown_2 : 2;    /**< \brief \internal unknown */
		CAN_RTR_t RTR : 1;             /**< \brief [6:6] RTR, Remote Transmission Request */
		CAN_frame_format_t FF : 1;     /**< \brief [7:7] Frame Format, see# CAN_frame_format_t*/
		unsigned int reserved_24 : 24; /**< \brief \internal Reserved */
	} B;
} CAN_FIR_t;

/** \brief CAN Frame structure */
typedef struct {
	CAN_FIR_t FIR;  /**< \brief Frame information record*/
	uint32_t MsgID; /**< \brief Message ID */
	union {
		uint8_t u8[8];   /**< \brief Payload byte access*/
		uint32_t u32[2]; /**< \brief Payload u32 access*/
		uint64_t u64;    /**< \brief Payload u64 access*/
	} data;
} CAN_frame_t;

void can_init ();
void can_deinit();
void can_send (CAN_frame_t *frame, uint8_t bus);
boolean can_receive_nonblocked (CAN_frame_t *rx_frame);
boolean can_receive_blocked (CAN_frame_t *rx_frame);
String canFrameToString(CAN_frame_t *frame);

#endif
