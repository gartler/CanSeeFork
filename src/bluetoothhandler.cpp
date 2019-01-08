#include "bluetoothhandler.h"

CS_CONFIG *bluetooth_config;
BluetoothSerial SerialBT;
void (*bluetooth_process)();

void bluetooth_init (CS_CONFIG *config, void (*p)()) {
  bluetooth_config = config;
  bluetooth_process = p;
  if (bluetooth_config->mode_bluetooth) {
    if (bluetooth_config->mode_debug) Serial.println("Bluetooth " + String (bluetooth_config->name_bluetooth) + " starting ...");
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
      bluetooth_process ();
      readBuffer = "";
    }
  } else {
    readBuffer += ch;
  }
}
