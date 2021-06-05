/**
* @file canhandler.cpp
* @brief This module will handle interfacing with the CANbus.
*/

#include "canhandler.h"

static CS_CONFIG_t *can_config;
static uint8_t used_bus;
CAN_device_t CAN_cfg;

/**
 * can_bus_set() initializes the settings for the CANbus subsystem
 */
void can_bus_set()
{
	if (used_bus == 0)
	{ // init the CAN bus (pins and baudrate)
		CAN_cfg.speed = (CAN_speed_t)can_config->can0_speed;
		CAN_cfg.tx_pin_id = (gpio_num_t)can_config->can0_tx;
		CAN_cfg.rx_pin_id = (gpio_num_t)can_config->can0_rx;
	}
	else
	{
		CAN_cfg.speed = (CAN_speed_t)can_config->can1_speed;
		CAN_cfg.tx_pin_id = (gpio_num_t)can_config->can1_tx;
		CAN_cfg.rx_pin_id = (gpio_num_t)can_config->can1_rx;
	}
}

/**
 * can_init() gets the config, sets the bus to 0, creates a freeRTOS queue for
 * the incoming frames and initializes the hardware
 */
void can_init()
{
	can_config = getConfig();
	used_bus = 0;
	can_bus_set();
	// create a generic RTOS queue for CAN receiving, with 10 positions
	CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
	if (CAN_cfg.rx_queue == 0)
	{
		if (can_config->mode_debug)
			writeOutgoingSerialDebug("Can't create CANbus buffer. Stopping");
		while (1)
			;
	}
	if (can_config->mode_debug)
		writeOutgoingSerialDebug("CANbus started");
	ESP32Can.CANInit(); // initialize CAN Module
}

/**
 * can_send() sends a frame directly to the hardware (no buffering). It might
 * reinitialize the hardware if a bus switch is requested, then writes the
 * frame
 * @param frame Pointer to the frame to be sent
 * @param bus Bus setting. At this moment, only 0 is allowed
 */
void can_send(CAN_frame_t *frame, uint8_t bus)
{
	if (bus != used_bus)
	{
		used_bus = bus;
		ESP32Can.CANStop();
		can_bus_set();
		ESP32Can.CANInit(); // initialize CAN Module
	}
	ESP32Can.CANWriteFrame(frame);
}

/**
 * can_receive is a non blocking function, fetching a frame is one is available
 * on the queue. Note that the queue is fed by the CANbus hardware ISR.
 * @param rx_frame Pointer to the frame that will be populated
 * @returns true if there was a frame available
 */
boolean can_receive(CAN_frame_t *rx_frame)
{
	return xQueueReceive(CAN_cfg.rx_queue, rx_frame, (TickType_t)0) == pdTRUE ? true : false;
}

/**
 * Convert a CAN_frame to readable hex output format
 * @param frame Pointer to the frame that will be populated
 * @returns String object representing the frame
 */
String canFrameToString(CAN_frame_t *frame)
{
	String dataString = String(frame->MsgID, HEX) + ",";
	for (int i = 0; i < frame->FIR.B.DLC; i++)
	{
		dataString += getHex(frame->data.u8[i]);
	}
	dataString += "\n";
	return dataString;
}
