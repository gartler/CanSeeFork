/**
* @file canhandler.cpp
* @brief This module will handle interfacing with the CANbus.
*/

#include "canhandler.h"

static CS_CONFIG_t *can_config;

/**
 * can_init() gets the config, sets the bus to 0, creates a freeRTOS queue for
 * the incoming frames and initializes the hardware
 */
void can_init()
{
	can_config = getConfig();
	can_filter_config_t filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
	can_timing_config_t timing_config;
	esp_err_t error;
	can_general_config_t general_config = {
		.mode = CAN_MODE_NORMAL,
		.tx_io = (gpio_num_t)can_config->can0_tx,
		.rx_io = (gpio_num_t)can_config->can0_rx,
		.clkout_io = (gpio_num_t)CAN_IO_UNUSED,
		.bus_off_io = (gpio_num_t)CAN_IO_UNUSED,
		.tx_queue_len = 20,
		.rx_queue_len = 20,
		.alerts_enabled = CAN_ALERT_NONE,
		.clkout_divider = 0};

	switch (can_config->can0_speed)
	{
	case CAN_SPEED_1000KBPS:
		timing_config = CAN_TIMING_CONFIG_1MBITS();
		break;
	case CAN_SPEED_500KBPS:
		timing_config = CAN_TIMING_CONFIG_500KBITS();
		break;
	case CAN_SPEED_250KBPS:
		timing_config = CAN_TIMING_CONFIG_250KBITS();
		break;
	}

	error = can_driver_install(&general_config, &timing_config, &filter_config);
	if (error != ESP_OK)
	{
		writeOutgoingSerialDebug("CAN Driver installation fail");
		return;
	}

	// start CAN driver
	error = can_start();
	if (error != ESP_OK)
	{
		writeOutgoingSerialDebug("CAN Driver start fail");
	}
}

/**
 * deinitialize canbus driver
 * 
 */
void can_deinit()
{
	//Stop the CAN driver
	if (can_stop() == ESP_OK)
	{
		writeOutgoingSerialDebug("Driver stopped\n");
	}
	else
	{
		writeOutgoingSerialDebug("Failed to stop driver\n");
		return;
	}

	//Uninstall the CAN driver
	if (can_driver_uninstall() == ESP_OK)
	{
		writeOutgoingSerialDebug("Driver uninstalled\n");
	}
	else
	{
		writeOutgoingSerialDebug("Failed to uninstall driver\n");
		return;
	}
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
	esp_err_t result;
	can_message_t native_frame;

	native_frame.data[0] = frame->data.u8[0];
	native_frame.data[1] = frame->data.u8[1];
	native_frame.data[2] = frame->data.u8[2];
	native_frame.data[3] = frame->data.u8[3];
	native_frame.data[4] = frame->data.u8[4];
	native_frame.data[5] = frame->data.u8[5];
	native_frame.data[6] = frame->data.u8[6];
	native_frame.data[7] = frame->data.u8[7];
	native_frame.data_length_code = frame->FIR.B.DLC;
	native_frame.flags = frame->FIR.B.FF == CAN_frame_std ? CAN_MSG_FLAG_NONE : CAN_MSG_FLAG_EXTD;
	native_frame.identifier = frame->MsgID;
	//writeOutgoingSerialDebug("can transmit");
	result = can_transmit(&native_frame, pdMS_TO_TICKS(20));
	if (result != ESP_OK)
	{
		writeOutgoingSerialDebug("can_send error:" + String(result));
	}
}

boolean can_receive_core(CAN_frame_t *rx_frame, TickType_t ticks_to_wait)
{
	can_message_t native_frame;

	if (can_receive(&native_frame, ticks_to_wait) == ESP_OK)
	{
		rx_frame->data.u8[0] = native_frame.data[0];
		rx_frame->data.u8[1] = native_frame.data[1];
		rx_frame->data.u8[2] = native_frame.data[2];
		rx_frame->data.u8[3] = native_frame.data[3];
		rx_frame->data.u8[4] = native_frame.data[4];
		rx_frame->data.u8[5] = native_frame.data[5];
		rx_frame->data.u8[6] = native_frame.data[6];
		rx_frame->data.u8[7] = native_frame.data[7];
		rx_frame->FIR.B.DLC = native_frame.data_length_code;
		rx_frame->FIR.B.FF = native_frame.flags & CAN_MSG_FLAG_EXTD ? CAN_frame_ext : CAN_frame_std;
		rx_frame->MsgID = native_frame.identifier;
		return true;
	}
	return false;
}

/**
 * can_receive is a non blocking function, fetching a frame is one is available
 * on the queue. Note that the queue is fed by the CANbus hardware ISR.
 * @param rx_frame Pointer to the frame that will be populated
 * @returns true if there was a frame available
 */
boolean can_receive(CAN_frame_t *rx_frame)
{
	return can_receive_core(rx_frame, (TickType_t)0);
}

/**
 * can_receive_blocled is a blocking function, waiting for a frame to be available
 * on the queue. Note that the queue is fed by the CANbus hardware ISR.
 * @param rx_frame Pointer to the frame that will be populated
 * @returns true if there was a frame available
 */
boolean can_receive_blocked(CAN_frame_t *rx_frame)
{
	return can_receive_core(rx_frame, portMAX_DELAY);
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
	return dataString;
}
