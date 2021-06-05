/**
* @file serialhandler.cpp
* @brief This module will handle serial communications.
*/

#include "serialhandler.h"

static CS_CONFIG_t *serial_config;

/**
 * Initialize serial subsystem
 */
void serial_init()
{ // Serial is already running
	serial_config = getConfig();
	if (!serial_config->mode_serial && !serial_config->mode_debug)
		Serial.end();
}

/**
 * Write a String to the serial output, if the serial mode is enabled
 * @param o String to send
 */
void writeOutgoingSerial(String o)
{
	if (!serial_config->mode_serial)
		return;
	Serial.println(o);
	if (serial_config->mode_debug & DEBUG_FLUSH_SERIAL)
		Serial.flush();
}

/**
 * Write a String to the serial output, if any debug mode is enabled
 * @param o String to send
 */
void writeOutgoingSerialDebug(String o)
{
	if (serial_config->mode_debug)
		return;
	Serial.println(o);
	if (serial_config->mode_debug & DEBUG_FLUSH_SERIAL)
		Serial.flush();
}

/**
 * Reads characters from the serial input to a supplied String
 * if a newline or return is received, the command handler will
 * be called and the buffer is cleared
 * @param readBuffer String to store command characters in
 */
void readIncomingSerial(String &readBuffer)
{
	if (!serial_config->mode_serial)
		return;
	if (!Serial.available())
		return;
	char ch = Serial.read();
	if (ch == '\n' || ch == '\r')
	{
		if (readBuffer != "")
		{
			if (serial_config->command_handler)
				serial_config->command_handler();
			readBuffer = "";
		}
	}
	else
	{
		readBuffer += ch;
	}
}
