#include "serialhandler.h"

static CS_CONFIG_t *serial_config;
static void (*serial_process)();

void serial_init (CS_CONFIG_t *config, void (*p)()) {              // Serial is already running
  serial_config = config;
  serial_process = p;
  if (!serial_config->mode_serial && !serial_config->mode_debug) Serial.end();
}

void writeOutgoingSerial (String o) {
  if (serial_config->mode_serial || serial_config->mode_debug) Serial.print (o);
}

void readIncomingSerial (String &readBuffer) {
  if (!serial_config->mode_serial && !serial_config->mode_debug) return;
  if (!Serial.available()) return;
  char ch = Serial.read();
  if (ch == '\n' || ch == '\r') {
    if (readBuffer != "") {
      serial_process ();
      readBuffer = "";
    }
  } else {
    readBuffer += ch;
  }
}
