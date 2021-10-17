/**
* @file bluetoothhandler.cpp
* @brief This module will handle bluetooth communications.
*/

#include "bluetoothhandler.h"
#include "leds.h"

static CS_CONFIG_t *bluetooth_config;
static BluetoothSerial SerialBT;
static bool bluetooth_active = false;

/**
 * watchDogStatus can be in two states:
 * value = 0: the watchdog is inactive
 * value > 1: the watchdog is counting up every 5 seconds.
 * The bluetooth watchdog can optionally be (re)started when a full command is received (readIncomingBluetooth)
 * setting it to 1.
 */
static uint16_t watchDogStatus = 0;

/**
 * Initialize bluetooth subsystem
 */
void bluetooth_init()
{
	bluetooth_config = getConfig();
	bluetooth_active = bluetooth_config->mode_bluetooth;
	if (bluetooth_active)
	{
		if (bluetooth_config->mode_debug & DEBUG_COMMAND)
			writeOutgoingSerialDebug("Bluetooth '" + String(bluetooth_config->name_bluetooth) + "' started.");
		SerialBT.begin(bluetooth_config->name_bluetooth); // init Bluetooth serial, no password in current framework
	}
}

/**
 * Simple Watchdog that is triggered every 5 seconds by the main loop. If the bluetooth
 * has a connection and the watchdog is started (optionally when a full command is received)
 * and it expires after aprx 35 seconds, the controller is rebooted
 */
void bluetoothWatchdogTicker()
{
	if (!bluetooth_active)
		return; // bluetooth should be active
	if (watchDogStatus == 0)
		return; // the watchdog should have been started (at least one line received)
	if (++watchDogStatus > 7)
		// if between 30 and 35 seconds no command at all
		// we might be in a race condition. BT is still connected, but SerialBT itself is dead
		// no other good solution found, so unfortunately reboot.
		ESP.restart();
}

bool hasClient()
{
	if (!bluetooth_active)
		return false; // bluetooth should be active
	if (SerialBT.hasClient())
	{
		bluetooth_config->mode_bluetooth = 2;
		return true;
	}
	bluetooth_config->mode_bluetooth = 1;
	return false;
}

/**
 * (dis)ables bluetooth communication
 */
void setActiveBluetooth(bool active)
{
	// If active is false in and output is simply ignored. At the moment This
	// is a safer solution than switching off Bluetooth or BluetoothSerial as
	// there seems to be a nasty memory leak in that part of the core, see
	// https://github.com/espressif/arduino-esp32/issues/1798
	if (!bluetooth_config->mode_bluetooth)
		return;
	bluetooth_active = active;
}

/**
 * Write a String to the bluetooth output, if it's active and connected
 * @param o String to send
 */
void writeOutgoingBluetooth(String o)
{
	if (hasClient())
		SerialBT.print(o);
}

/**
 * Reads characters from the bluetooth input to a supplied String
 * if a newline or return is received, the command handler will
 * be called and the buffer is cleared
 * @param readBuffer String to store command characters in
 */
void readIncomingBluetooth(String &readBuffer)
{
	static bool hadClient = false;

	if (!bluetooth_config->mode_bluetooth)
		return;

	if (hasClient())
	{
		led_set(LED_BLUE, true);

		if (!hadClient)
		{
			writeOutgoingSerialDebug("Bluetooth connected");
			hadClient = true;
		}
	}
	else
	{
		led_set(LED_BLUE, false);

		if (hadClient)
		{
			writeOutgoingSerialDebug("Bluetooth '" + String(bluetooth_config->name_bluetooth) + "' restarted.");
			SerialBT.end();
			SerialBT.begin(bluetooth_config->name_bluetooth);
			hadClient = false;
		}
	}

	if (!SerialBT.available())
		return;
	char ch = SerialBT.read();
	if (ch == '\n' || ch == '\r')
	{
		if (readBuffer != "")
		{
			if (bluetooth_active && bluetooth_config->command_handler)
				bluetooth_config->command_handler();
			// watchDogStatus = 1; //(re)start the watchdog
			readBuffer = "";
		}
	}
	else
	{
		readBuffer += ch;
	}
}
