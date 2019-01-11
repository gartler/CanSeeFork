#include "bluetoothhandler.h"

static CS_CONFIG_t *bluetooth_config;
static BluetoothSerial SerialBT;

void bluetooth_init () {
  bluetooth_config = getConfig ();
  if (bluetooth_config->mode_bluetooth) {
    if (bluetooth_config->mode_debug) Serial.println("Bluetooth '" + String (bluetooth_config->name_bluetooth) + "' started.");
    SerialBT.begin(bluetooth_config->name_bluetooth); // init Bluetooth serial, no password in current framework
  }
}

void writeOutgoingBluetooth (String o) {
  if (bluetooth_config->mode_bluetooth && SerialBT.hasClient()) SerialBT.print (o);
}

void readIncomingBluetooth (String &readBuffer) {
  if (!bluetooth_config->mode_bluetooth) return;
  led_set (LED_BLUE, SerialBT.hasClient());
  if (!SerialBT.available()) return;
  char ch = SerialBT.read();
  if (ch == '\n' || ch == '\r') {
    if (readBuffer != "") {
      if (bluetooth_config->command_handler) bluetooth_config->command_handler ();
      readBuffer = "";
    }
  } else {
    readBuffer += ch;
  }
}
