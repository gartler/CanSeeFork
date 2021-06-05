/**
* @file freeframehandler.cpp
* @brief This module will handle incoming free frames.
*/
#include "freeframehandler.h"

static CS_CONFIG_t *freeframe_config;
static FREEFRAME_t freeframes[FREEFRAMEARRAYSIZE];

/**
 * initializes the subsystem by clearing out the freeframe buffer
 */
void freeframe_init()
{
	freeframe_config = getConfig();
	for (int id = 0; id < FREEFRAMEARRAYSIZE; id++)
	{ // clear the free frame buffer. Data zeroed, length zeroed, not timed out
		for (int i = 0; i < 8; i++)
			freeframes[id].data[i] = 0;
		freeframes[id].length = 0;
		freeframes[id].age = 6; // allow 25-30 seconds after boot to display empty frames
	}
}

/**
 * storeFreeFrame stores a received free frame in the budder for that ID. This way, the
 * last frame received can always be requested without any blocking
 */
void storeFreeframe(CAN_frame_t *frame, uint8_t bus)
{
	if (!(frame->MsgID < FREEFRAMEARRAYSIZE))
		return;
	if (freeframe_config->mode_debug & DEBUG_BUS_RECEIVE_FF)
	{
		writeOutgoingSerialDebug("< can:FF:");
		writeOutgoingSerialDebug(canFrameToString(frame));
	}
	for (int i = 0; i < 8; i++)
	{ // store a copy
		freeframes[frame->MsgID].data[i] = frame->data.u8[i];
	}
	freeframes[frame->MsgID].length = frame->FIR.B.DLC; // and the length
	freeframes[frame->MsgID].age = 2;					// age in to 5-10 seconds
}

/**
 * ageFreeFrame is called every 5 seconds to age all buffered frames. If a
 * frame reaches age 0 (counting down), it will be invalidated.
 */
void ageFreeFrame()
{
	for (int id = 0; id < FREEFRAMEARRAYSIZE; id++)
	{ // clear the free frame buffer. Data zeroed, length zeroed, not timed out
		if (freeframes[id].age > 0)
			freeframes[id].age--;
	}
}

/**
 * getFreeFrame fetches a frame from the buffer.
 * @param id ID of the requested frame
 * @param bus bus the frame was received on (only 0 allowed)
 * @returns pointer to the buffered frame
 */
FREEFRAME_t *getFreeframe(uint32_t id, uint8_t bus)
{
	if (id >= FREEFRAMEARRAYSIZE)
		return &freeframes[0];
	return &freeframes[id];
}

/**
 * requestFreeframe sends the contents of the requested frame to the output
 * @param id ID of the requested frame
 * @param bus bus the frame was received on (only 0 allowed)
 */
void requestFreeframe(uint32_t id, uint8_t bus)
{
	if (freeframe_config->output_handler)
	{
		freeframe_config->output_handler(bufferedFrameToString(id, bus));
	}
}

/**
 * bufferedFrameToString converts a buffered frame to readable hex output format, newline terminated
 * @param id ID of the requested frame
 * @param bus bus the frame was received on (only 0 allowed)
 * @returns \n terminated string representing the frmae
 */
String bufferedFrameToString(uint32_t id, uint8_t bus)
{
	String dataString = String(id, HEX) + ",";
	if (freeframes[id].age)
	{ // do not output stale data
		for (int i = 0; i < freeframes[id].length; i++)
		{
			dataString += getHex(freeframes[id].data[i]);
		}
	}
	dataString += "\n";
	return dataString;
}
