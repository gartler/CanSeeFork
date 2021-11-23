/**
* @file bluetoothhandler.cpp
* @brief This module will handle bluetooth communications.
*/

#include "bluetoothhandler.h"
#include "leds.h"

#include "esp_gap_bt_api.h"


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
 * Specify a different Class Of Device, to allow connections on Chrome OS
 */
static void bluetoothSetClassOfDevice()
{
	// Copy/paste from: https://github.com/espressif/arduino-esp32/blob/master/libraries/BluetoothSerial/src/BluetoothSerial.cpp - Line 636
	// This should change the announced BT Class of Device (COD) to something Chrome OS will accept...
	// - - -
	// the default BTA_DM_COD_LOUDSPEAKER does not work with the macOS BT stack
	esp_bt_cod_t cod;
	// Settings from source:
	//cod.major = 0b00001;			// 0b00001 = Computer (desktop,notebook, PDA, organizers, .... )
	//cod.minor = 0b000100;			// 0b000100 = Handheld PC/PDA (clam shell)
	//cod.service = 0b00000010110;	// Networking (LAN, Ad hoc, ...) + 2*reserved
	// Modified settings, copied from KONNWEI. It now looks the same on Android, but still no serial connection from Chromebook
	// See https://flylib.com/books/en/1.134.1/the_bluetooth_protocol.html
	// and ask the entire class identifier using crosh command bt_console, then devices to find the mac, and info [mac] to see details
	cod.major = 0b11111;			// 0x1f: Uncategorized
	cod.minor = 0b000000;			// 0x00: Uncategorized device
	cod.service = 0b00000000000;	// 
	if (esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD) != ESP_OK) {
		writeOutgoingSerialDebug("Bluetooth - Set cod failed");
	}
	// - - -
}

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
		bluetoothSetClassOfDevice();
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
			bluetoothSetClassOfDevice();
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
