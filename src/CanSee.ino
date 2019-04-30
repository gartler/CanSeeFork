/*
CanSee
The firmware to the DIY, superfast, ESP32 based comapanion to CANZE dongle

Copyright (C) 2019 - The CanZE Team
http://canze.fisch.lu

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or any
later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define VERSION "002"

#define SERIAL_BPS 115200

/* board is a CP2102 like this
https://www.ebay.com.au/itm/262903668612 or
https://www.tinytronics.nl/shop/en/communication/network/esp32-wi-fi-and-bluetooth-board-cp2102

My board: DOIT ESP32 DEVKIT V1

ESP32 GPIO5 - CAN CTX
ESP32 GPIO4 - CAN CRX

ESP32 GPIO26  white  LED with 3.3 kO resistor
ESP32 GPIO27  yellow LED with 120 O  resistor
ESP32 GPIO32  red    LED with 120 O  resistor
ESP32 GPIO33  green  LED with 120 O  resistor
ESP32 GPIO25  blue   LED with 120 O  resistor
*/

// Our own includes, see ./include/ ******************************************
#include "config.h"
#include "canhandler.h"
#include "leds.h"
#include "serialhandler.h"
#include "bluetoothhandler.h"
#include "wifihandler.h"
#include "freeframehandler.h"
#include "isotphandler.h"
#include "utils.h"

// Tidy up defs **************************************************************
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Config ********************************************************************
CS_CONFIG_t *cansee_config;

// Command structure that defines a textual incoming command *****************
typedef struct {
  char cmd;
  uint32_t id = 0;
  uint8_t request[32];
  uint16_t requestLength = 0;
  uint8_t reply[32];
  uint16_t replyLength = 0;
  char line[48];
} COMMAND_t;

// command read buffer *******************************************************
String readBuffer = "";

//* counters *****************************************************************
uint32_t canFrameCounter = 1;
uint32_t lastCanFrameCounter = 0;
uint32_t cnt5000 = 0;
uint32_t cnt100 = 0;

// ***************************************************************************
void setup() {
  Serial.begin (SERIAL_BPS);                         // init serial
  Serial.println ("");
  Serial.println ("");
  Serial.println ("CANSee starting...");

  delay (500);                                     // give user chance to press BUT
  pinMode (0, INPUT);
  cansee_config = getConfig ();                    // invalid config will reset too
  if (!digitalRead (0)) {                          // if pressed
    Serial.println("Reset config...");
    setConfigToEeprom (true);
  }
  cansee_config->output_handler = writeOutgoing;
  cansee_config->command_handler = processCommand;

  if (cansee_config->mode_debug) {
    Serial.print   ("Version:   "); Serial.println (VERSION);
    Serial.println ("Serial:    " + getHex (cansee_config->mode_serial   ));
    Serial.println ("Bluetooth: " + getHex (cansee_config->mode_bluetooth));
    Serial.println ("WiFi:      " + getHex (cansee_config->mode_wifi     ));
    Serial.println ("Leds:      " + getHex (cansee_config->mode_leds     ));
    Serial.println ("Debug:     " + getHex (cansee_config->mode_debug    ));
    Serial.println ("CANbus0:   " + getHex (cansee_config->can0_speed / 25) + getHex (cansee_config->can0_rx) + getHex (cansee_config->can0_tx));
    Serial.println ("CANbus1*   " + getHex (cansee_config->can1_speed / 25) + getHex (cansee_config->can1_rx) + getHex (cansee_config->can1_tx));
    Serial.println ("Boot count:" + getHex (cansee_config->boot_count    ));
  }
  cansee_config->boot_count++;
  setConfigToEeprom (false);

  leds_init ();

  serial_init ();
  bluetooth_init ();
  wifi_init ();

  can_init ();
  freeframe_init ();
  isotp_init ();

}

// ***************************************************************************
void loop() {
  tickerFast ();
}

void tickerFast () {
  uint32_t nowMicros = micros ();
  static uint32_t lastMicros = nowMicros;           // static so should only be initalized once

  // do Fast
  CAN_frame_t rx_frame;                            // 1. receive next CAN frame from queue
  if (can_receive (&rx_frame)) {
    storeFrame (rx_frame);
    canFrameCounter++;
  }
  readIncoming ();                                 // 2. proceed with input (serial & BT)
  isotp_ticker ();
  // end do Fast

  if ((nowMicros - lastMicros) > 100000L) { // 110 ms passed?
    ticker100ms ();
    //lastMicros = nowMicros;
    lastMicros += 100000L;
  }
}

void ticker100ms () {
  static int tick = 0;

  // do every 100ms
  // Things like button pushed should go here
  // end do every 100 ms

  if (++tick == 10) {
    ticker1000ms ();
    tick = 0;
  }
}

void ticker1000ms () {
  static int tick = 0;

  // do every 1000ms
  static bool powerToggle = false;
  led_set(LED_RED, (powerToggle = !powerToggle));
  // end do every 1000 ms

  if (++tick == 5) {
    ticker5000ms ();
    tick = 0;
  }
}

void ticker5000ms () {
  // do every 5000ms
  setActiveBluetooth (canFrameCounter != lastCanFrameCounter);
  ageFreeFrame ();
  // end do every 5000 ms
}

/*****************************************************************************
* frame handling function
*/

void storeFrame (CAN_frame_t &frame) {
  led_set (LED_GREEN, true);
  if (frame.MsgID < 0x700) {                      // free data is < 0x700
    storeFreeframe (frame, 0);
  } else if (frame.MsgID < 0x800) {                // iso-tp data is < 0x800
    storeIsotpframe (frame, 0);
  }
  led_set (LED_GREEN, false);
}

// I/O functions *************************************************************

void writeOutgoing (String o) {
  writeOutgoingSerial (o);
  writeOutgoingBluetooth (o);
  writeOutgoingWiFi (o);
}

void readIncoming() {
  readIncomingSerial (readBuffer);
  readIncomingBluetooth (readBuffer);
  readIncomingWiFi (readBuffer);
}

// execute a command *********************************************************
void processCommand () {
  uint8_t bus = 0;

  if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println("< com:" + readBuffer);
  COMMAND_t command = decodeCommand (readBuffer);    // watch out, passe dby reference, so eaten

  //bus = (command.id & 0x40000000) ? 1 : 0;
  //command.id &= 0x1fffffff;                        // allow for 29 bits CAN later

  switch (command.cmd) {

    // output all buffered frames ********************************************
    case 'a': {
      int count = 0;
      FREEFRAME_t *freeframe;
      for (uint32_t id = 0; id < FREEFRAMEARRAYSIZE; id++) {
        freeframe = getFreeframe (id, bus);        // bus is ignored for free frames
        if (freeframe->age) {                      // print length 0 frames, but do not print timeout frames
          //writeOutgoing (bufferedFrameToString (id, bus)); // includes \n
          requestFreeframe (id, bus);
          count++;
        }
      }
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println("> fcn:" + String (count));
    }
    break;

    // get a frame ***********************************************************
    case 'g':
    if (command.id < FREEFRAMEARRAYSIZE) {
      requestFreeframe (command.id, bus);
    } else {
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.print ("> com:ID out of bounds (0 - 0x6ff)");
      requestFreeframe (0, bus);
    }
    break;

    // request an ISO-TP frame ***********************************************
    case 'i':
    if (command.id >= 0x700 && command.id <= 0x7ff) {
      requestIsotp (command.id, command.requestLength, command.request, bus);
    } else {
      if (cansee_config->mode_debug) Serial.println ("E:ID out of bounds (0x700 - 0x7ff)");
      requestIsotp (0, 0, 0, 0);
    }
    break;

    // inject a frame via serial / BT input **********************************
    case 't': {
      CAN_frame_t frame;
      frame.MsgID = command.id;
      frame.FIR.B.DLC = command.requestLength;
      for (int i = 0; i < command.requestLength; i++)
      frame.data.u8[i] = command.request[i];
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.print ("> com:Injecting " + canFrameToString (frame));
      storeFrame (frame);
      // storeframe will output if free frame or ISO-TP Single
      // writeOutgoing (getHex (command.id) + "\n");
    }
    break;

    // filter (deprecated) ***************************************************
    case 'f':
    if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println ("> com:Filter " + getHex (command.id));
    writeOutgoing (getHex (command.id) + "\n");
    break;

    // config (see config.cpp) ***********************************************
    case 'n':
    if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println ("> com:config " + getHex (command.id));
    switch (command.id) {
      case 0x100: // set mode flags
      cansee_config->mode_serial     = command.request [0];
      cansee_config->mode_bluetooth  = command.request [1];
      cansee_config->mode_wifi       = command.request [2];
      cansee_config->mode_leds       = command.request [3];
      cansee_config->mode_debug      = command.request [4];
      break;
      case 0x101: // get mode flags
      writeOutgoing (getHex (command.id) + "," + getHex (cansee_config->mode_serial) + getHex (cansee_config->mode_bluetooth) + getHex (cansee_config->mode_wifi) + getHex (cansee_config->mode_leds) + getHex (cansee_config->mode_debug) + "\n");
      return;
      break;
      case 0x200:
      strncpy (cansee_config->name_bluetooth, command.line + 5, sizeof (cansee_config->name_bluetooth));
      break;
      case 0x201:
      strncpy (cansee_config->pin_bluetooth, command.line + 5, sizeof (cansee_config->pin_bluetooth));
      break;
      case 0x300:
      strncpy (cansee_config->ssid_ap, command.line + 5, sizeof (cansee_config->ssid_ap));
      break;
      case 0x301:
      strncpy (cansee_config->password_ap, command.line + 5, sizeof (cansee_config->password_ap));
      break;
      case 0x400:
      strncpy (cansee_config->ssid_station, command.line + 5, sizeof (cansee_config->ssid_station));
      Serial.println (sizeof (cansee_config->ssid_station));
      Serial.println (command.line);
      Serial.println (cansee_config->ssid_station);
      break;
      case 0x401:
      strncpy (cansee_config->password_station, command.line + 5, sizeof (cansee_config->password_station));
      break;
      case 0x500: // can0
      cansee_config->can0_speed      = command.request [0] * 25;
      cansee_config->can0_rx         = command.request [1];
      cansee_config->can0_tx         = command.request [2];
      case 0x501: // can1
      cansee_config->can1_speed      = command.request [0] * 25;
      cansee_config->can1_rx         = command.request [1];
      cansee_config->can1_tx         = command.request [2];
      break;
    }
    setConfigToEeprom (false);
    writeOutgoing (getHex (command.id) + "\n");
    break;

    // reboot *****************************************************************
    case 'z':
    ESP.restart();
    break;

    // reset config & reboot **************************************************
    case 'r':
    setConfigToEeprom(true);
    ESP.restart();
    break;

    // give up ****************************************************************
    default:
    if (cansee_config->mode_debug) Serial.println ("> com:Unknown command " + String (command.cmd));
    writeOutgoing("fff,\n");
    break;
  }
}

// parse a string into a command *********************************************
COMMAND_t decodeCommand (String &input) {
  COMMAND_t result;
  result.id = 0;                                     // clear out older data
  result.requestLength = 0;
  result.replyLength = 0;

  input.trim();                                      // trim whitespaces
  strncpy (result.line, input.c_str(), sizeof (result.line));

  if (input.length() == 0) return result;            // stop if input is empty

  result.cmd = input.charAt(0);                      // the first letter is the command
  input.remove(0, 1);

  if (input.length() != 0) {                         // if there is something more,
    char ch;                                         // get the ID
    String id = "";
    do {
      ch = input.charAt(0);
      if (ch != ',') id += ch;
      input.remove(0, 1);
    } while (input.length() != 0 && ch != ',');
    result.id = hexToDec(id);
  }

  if (input.length() != 0) {                         // if there is something more,
    char ch;                                         // get the REQUEST
    String request = "";
    do {
      ch = input.charAt(0);
      if (ch != ',') request += ch;
      input.remove(0, 1);
    } while (input.length() != 0 && ch != ',');
    for (int i = 0; i < request.length() && result.requestLength < 32; i += 2) {// check for overflow
      result.request[result.requestLength] = hexToDec(request.substring(i, i + 2));
      result.requestLength++;
    }
  }

  if (input.length() != 0) {                         // if there is something more,
    char ch;                                         // get the REPLY
    String reply = "";
    do {
      ch = input.charAt(0);
      if (ch != ',') reply += ch;
      input.remove(0, 1);
    } while (input.length() != 0 && ch != ',');
    for (int i = 0; i < reply.length() && result.replyLength < 32; i += 2) { // check for overflow
      result.reply[result.replyLength] = hexToDec(reply.substring(i, i + 2));
      result.replyLength++;
    }
  }
  return result;
}
