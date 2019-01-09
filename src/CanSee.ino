/*
CanSee
The firmware to the DIY, superfase, ESP32 based comapanion to CANZE dongle

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
#define DEBUG_BUS_RECEIVE_FF  0x01
#define DEBUG_BUS_RECEIVE_ISO 0x02
#define DEBUG_COMMAND         0x04
#define DEBUG_COMMAND_FF      0x08
#define DEBUG_COMMAND_ISO     0x10

#define VERSION "001"

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
  uint8_t request[8];
  uint8_t requestLength = 0;
  uint8_t reply[32];
  uint8_t replyLength = 0;
  char line[48];
} COMMAND_t;

// ISO-TP message ************************************************************
typedef struct {
  uint16_t id = 0xffff;                            // the from-address of the responding device
  uint16_t requestId = 0xffff;                     // the to-address of the device to send the request to
  uint16_t length = 0;
  uint8_t index = 0;
  uint8_t next = 1;
  uint8_t request[8];
  uint8_t requestLength = 0;
  // uint8_t reply[8];
  // uint8_t replyLength = 0;
  // uint8_t* data = 0;
  uint8_t data[1024];
} ISO_MESSAGE_t;

// declare an ISO-TP message
ISO_MESSAGE_t isoMessage;

// command read buffer *******************************************************
String readBuffer = "";

// ***************************************************************************
void setup() {
  Serial.begin (SERIAL_BPS);                         // init serial
  Serial.println ("");
  Serial.println ("");
  Serial.println ("CANSee starting...");

  delay (500);                                      // give user chance to press BUT
  pinMode (0, INPUT);
  if (!digitalRead (0)) {                           // if pressed
    Serial.println("Reset config...");
    setConfigToEeprom (true);
  }
  cansee_config = getConfigFromEeprom ();           // invalid config will reset too

  if (cansee_config->mode_debug) {
    Serial.print   ("Version:   "); Serial.println (VERSION);
    Serial.println ("Serial:    " + String (cansee_config->mode_serial   , HEX));
    Serial.println ("Bluetooth: " + String (cansee_config->mode_bluetooth, HEX));
    Serial.println ("WiFi:      " + String (cansee_config->mode_wifi     , HEX));
    Serial.println ("Leds:      " + String (cansee_config->mode_leds     , HEX));
    Serial.println ("Debug:     " + String (cansee_config->mode_debug    , HEX));
  }

  leds_init (cansee_config);

  serial_init (cansee_config, processCommand);
  bluetooth_init (cansee_config, processCommand);
  wifi_init (cansee_config, processCommand);

  can_init (cansee_config);
  freeframe_init ();
}

// ***************************************************************************
void loop() {
  // 1. receive next CAN frame from queue
  // removed 3 * portTICK_PERIOD_MS to not block the code
  CAN_frame_t rx_frame;
  if (can_receive (&rx_frame)) {
    storeFrame (rx_frame);
  }

  // 2. proceed with input (serial & BT)
  readIncoming ();

  // 3. age data array of free frames
  // to do

}

/*****************************************************************************
* frame handling function
* see https://en.wikipedia.org/wiki/ISO_15765-2 for ISO-TP
*/

void storeFrame (CAN_frame_t &frame) {
  if (frame.MsgID < 0x700) {                      // free data stream is < 0x700
    led_set (LED_YELLOW, true);
    if (cansee_config->mode_debug & DEBUG_BUS_RECEIVE_FF) {
      Serial.print ("FF:");
      Serial.print(canFrameToString(frame));
    }
    storeFreeframe (frame);
    led_set (LED_YELLOW, false);
  }

  // if there is content and this is the frame we are waiting for
  else if (frame.FIR.B.DLC > 0 && frame.MsgID == isoMessage.id) {
    led_set (LED_WHITE, true);

    uint8_t type = frame.data.u8[0] >> 4;          // id = first nibble

    // single frame answer ***************************************************
    if (type == 0x0) {
      if (cansee_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) {
        Serial.print(">>ISO SING:");
        Serial.print(canFrameToString(frame));
      }

      uint16_t messageLength = frame.data.u8[0] & 0x0f;// length = second nibble + second byte
      if (messageLength > 7) messageLength = 7;    // this should never happen
      isoMessage.length = messageLength;

      // clear data buffer
      for (int i = 0; i < messageLength; i++)
      isoMessage.data[i] = 0;

      // fill up with this initial first-frame data (should always be 6)
      isoMessage.index = 0;                        // pointer at start of array
      for (int i = 1; i < frame.FIR.B.DLC; i++) {  // was starting at 4?
        if (isoMessage.index < isoMessage.length) {
          isoMessage.data[isoMessage.index++] = frame.data.u8[i];
        }
      }
      writeOutgoing (isoMessageToString (isoMessage));

      // cancel this message
      isoMessage.id = 0xffff;
    }

    // first frame of a multi-framed message *********************************
    else if (type == 0x1) {
      if (cansee_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) {
        Serial.print("ISO FRST:");
        Serial.print(canFrameToString(frame));
      }

      // start by requesting requesing the type Consecutive (0x2) frames by sending a Flow frame
      can_send_flow (isoMessage.requestId);

      // build new iso message
      // isoMessage.id = frame.MsgID;              // .id is already set when sending, and checked when answer was received
      // set final length
      uint16_t messageLength = frame.data.u8[0] & 0x0f;// length = second nibble + second byte
      messageLength <<= 8;
      messageLength |= frame.data.u8[1];
      if (messageLength > 1023) messageLength = 1023; // this should never happen
      isoMessage.length = messageLength;

      // clear data buffer
      for (int i = 0; i < messageLength; i++)
      isoMessage.data[i] = 0;

      // init sequence
      isoMessage.next = 1;                         // we are handling frame 0 now, so the next one is 1

      // fill up with this initial first-frame data (should always be 6)
      isoMessage.index = 0;                        // pointer at start of array
      for (int i = 2; i < frame.FIR.B.DLC; i++) {  // was starting at 4?
        if (isoMessage.index < isoMessage.length) {
          isoMessage.data[isoMessage.index++] = frame.data.u8[i];
        }
      }
    }

    // consecutive frame(s) **************************************************
    else if (type == 0x2) {
      if (cansee_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) {
        Serial.print("ISO NEXT:");
        Serial.print(canFrameToString(frame));
      }

      uint8_t sequence = frame.data.u8[0] & 0x0f;
      if (isoMessage.next == sequence) {
        for (int i = 1; i < frame.FIR.B.DLC; i++) {
          if (isoMessage.index < isoMessage.length) {
            isoMessage.data[isoMessage.index++] = frame.data.u8[i];
          }
        }

        // wait for next message, rollover from 15 to 0
        if (isoMessage.next++ == 15) isoMessage.next = 0;

        // is this the last part?
        if (isoMessage.index == isoMessage.length) {
          // output the data
          String dataString = isoMessageToString(isoMessage);
          if (cansee_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) Serial.print(">>ISO MSG:");
          writeOutgoing(dataString);

          // cancel this message
          isoMessage.id = 0xffff;
        }
      } else {
        if (cansee_config->mode_debug) Serial.println("E:ISO Out of sequence, resetting");
        //writeOutgoing("fff,\n");
        isoMessage.id = 0xffff;
      }
    } else {
      if (cansee_config->mode_debug) Serial.println("E:ISO ignoring unknown frame type:" + String (type));
    }

    led_set (LED_WHITE, false);
  } else {
    if (cansee_config->mode_debug) Serial.println("E:ISO frame of unrequested id:" + String(frame.MsgID, HEX));
  }
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
  uint8_t bus;

  if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println("< com:" + readBuffer);
  COMMAND_t command = decodeCommand (readBuffer);    // watch out, passe dby reference, so eaten

  bus = (command.id & 0x40000000) ? 1 : 0;
  command.id &= 0x1fffffff;                        // allow for 29 bits CAN later

  switch (command.cmd) {

    // output all buffered frames ********************************************
    case 'a': {
      int count = 0;
      FREEFRAME_t *freeframe;
      for (uint32_t id = 0; id < FREEFRAMEARRAYSIZE; id++) {
        freeframe = getFreeframe (id);
        if (freeframe->age) {                    // print length 0 frames, but do not print timeout frames
          writeOutgoing (bufferedFrameToString (id)); // includes \n
          count++;
        }
      }
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println("> fcn:" + String (count));
    }
    break;

    // get a frame ***********************************************************
    case 'g':
    if (command.id < FREEFRAMEARRAYSIZE) {
      writeOutgoing (bufferedFrameToString(command.id));
    } else {
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.print ("> com:ID out of bounds (0 - 0x6ff)");
      writeOutgoing (bufferedFrameToString(0));
    }
    break;

    // request an ISO-TP frame ***********************************************
    case 'i':
    // only accept this command if the requested ID belongs to an ISO-TP frame
    if (command.id < 0x700 || command.id > 0x7ff) {
      if (cansee_config->mode_debug) Serial.println ("E:ID out of bounds (0x700 - 0x7ff)");
      writeOutgoing (String (command.id, HEX) + "\n");
      return;
    }
    // store ID
    isoMessage.id = command.id;                    // expected ID of answer
    if ((isoMessage.requestId = getRequestId(command.id)) == 0) { // ID to send request to
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println ("> com:" + String (command.id, HEX) + " has no corresponding request ID");
      writeOutgoing (String (command.id, HEX) + "\n");
      return;
    }
    // store request
    isoMessage.requestLength = command.requestLength;
    if (isoMessage.requestLength > 7) isoMessage.requestLength = 7; // this should never happen
    for (int i = 0; i < command.requestLength; i++)
    isoMessage.request[i] = command.request[i];

    CAN_frame_t frame;                             // build the CAN frame
    frame.FIR.B.FF = CAN_frame_std;                // set the type to 11 bits
    frame.FIR.B.RTR = CAN_no_RTR;                  // no RTR
    frame.MsgID = isoMessage.requestId;            // set the ID
    frame.FIR.B.DLC = 8; //command.requestLength + 1; // set the length. Note some ECU's like DLC 8
    for (int i = 0; i < frame.FIR.B.DLC; i++)      // zero out frame
    frame.data.u8[i] = 0;

    // we are assuming here that requests are always single frame. This is formally not true, but good enough for us
    frame.data.u8[0] = (command.requestLength & 0x0f);

    for (int i = 0; i < command.requestLength; i++)// fill up the other bytes with the request
    frame.data.u8[i + 1] = command.request[i];

    // send the frame
    if (cansee_config->mode_debug & DEBUG_COMMAND_ISO) {
      Serial.print ("> com:Sending ISOTP request:");
      Serial.print (canFrameToString (frame));
    }
    can_send (&frame, bus);
    // --> any incoming frames with the given id will be handled by "storeFrame" and send off if complete
    break;

    // inject a frame via serial / BT input **********************************
    case 't': {
      CAN_frame_t frame;
      frame.MsgID = command.id;
      frame.FIR.B.DLC = command.requestLength;
      for (int i = 0; i < command.requestLength; i++)
      frame.data.u8[i] = command.request[i];
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.print ("> com:Injecting " + canFrameToString (frame));
      storeFrame(frame);
      // storeframe will output if free frame or ISO-TP Single
      // writeOutgoing (String (command.id, HEX) + "\n");
    }
    break;

    // filter (deprecated) ***************************************************
    case 'f':
    if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println ("> com:Filter " + String (command.id, HEX));
    writeOutgoing (String (command.id, HEX) + "\n");
    break;

    // config (see config.cpp) ***********************************************
    case 'n':
    if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println ("> com:config " + String (command.id, HEX));
    switch (command.id) {
      case 0x100: // set mode flags
      cansee_config->mode_serial     = command.request [0];
      cansee_config->mode_bluetooth  = command.request [1];
      cansee_config->mode_wifi       = command.request [2];
      cansee_config->mode_leds       = command.request [3];
      cansee_config->mode_debug      = command.request [4];
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
      case 0x500: // can1
      cansee_config->can1_speed      = command.request [0];
      cansee_config->can1_rx         = command.request [1];
      cansee_config->can1_tx         = command.request [2];
      case 0x501: // can2
      cansee_config->can2_speed      = command.request [0];
      cansee_config->can2_rx         = command.request [1];
      cansee_config->can2_tx         = command.request [2];
      break;
    }
    setConfigToEeprom (false);
    writeOutgoing (String (command.id, HEX) + "\n");
    break;

    // reset *****************************************************************
    case 'z':
    ESP.restart();
    break;

    // give up ***************************************************************
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
    for (int i = 0; i < request.length() && result.requestLength < 8; i += 2) {// check for overflow
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
    for (int i = 0; i < reply.length() && result.replyLength < 8; i += 2) { // check for overflow
      result.reply[result.replyLength] = hexToDec(reply.substring(i, i + 2));
      result.replyLength++;
    }
  }
  return result;
}


// convert a ISO-TP message to readable hex output format
String isoMessageToString(ISO_MESSAGE_t &message) {
  String dataString = String(message.id, HEX) + ",";
  for (int i = 0; i < message.length; i++) {
    dataString += getHex(message.data[i]);
  }
  dataString += "\n";
  return dataString;
}

// ZOE CAN computer related functions ****************************************
uint16_t getRequestId(uint16_t responseId) {
                      // from ECU id   to ECU id
  if      (responseId == 0x7ec) return 0x7e4; // EVC
  else if (responseId == 0x7da) return 0x7ca; // TCU
  else if (responseId == 0x7bb) return 0x79b; // LBC
  else if (responseId == 0x77e) return 0x75a; // PEB
  else if (responseId == 0x772) return 0x752; // Airbag
  else if (responseId == 0x76d) return 0x74d; // UDP
  else if (responseId == 0x763) return 0x743; // instrument panel
  else if (responseId == 0x762) return 0x742; // PAS
  else if (responseId == 0x760) return 0x740; // ABS
  else if (responseId == 0x7bc) return 0x79c; // UBP
  else if (responseId == 0x765) return 0x745; // BCM
  else if (responseId == 0x764) return 0x744; // CLIM
  else if (responseId == 0x76e) return 0x74e; // UPA
  else if (responseId == 0x793) return 0x792; // BCB
  else if (responseId == 0x7b6) return 0x796; // LBC2
  else if (responseId == 0x722) return 0x702; // LINSCH
  else if (responseId == 0x767) return 0x747; // AUTOS (R-LINK)
  else return 0;
}
