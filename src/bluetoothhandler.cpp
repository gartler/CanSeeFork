#include "bluetoothhandler.h"
#include "leds.h"

static CS_CONFIG_t *bluetooth_config;
static BluetoothSerial SerialBT;
static bool bluetooth_active = false;
static int16_t watchDogStatus = 0;

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

void bluetoothWatchdogTicker()
{
  if (!bluetooth_active)
    return; // bluetooth should be active
  if (watchDogStatus == 0)
    return; // the watchdod should have been started (at least one line received)
  if (++watchDogStatus > 7)
    // if between 30 and 35 seconds no command at all
    // we might be in a race condition. BT is still connected, but SerialBT itself is dead
    // no other good solution found, so unfortunately reboot.
    ESP.restart();
}

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

void writeOutgoingBluetooth(String o)
{
  if (bluetooth_active && SerialBT.hasClient())
    SerialBT.print(o);
}

void readIncomingBluetooth(String &readBuffer)
{
  if (!bluetooth_config->mode_bluetooth)
    return;
  led_set(LED_BLUE, SerialBT.hasClient());
  if (!SerialBT.available())
    return;
  char ch = SerialBT.read();
  if (ch == '\n' || ch == '\r')
  {
    if (readBuffer != "")
    {
      if (bluetooth_active && bluetooth_config->command_handler)
        bluetooth_config->command_handler();
      watchDogStatus = 1; // reset and start the watchdog
      readBuffer = "";
    }
  }
  else
  {
    readBuffer += ch;
  }
}
