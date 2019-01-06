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
#define DEBUG 1
//#define DEBUG_BUS_RECEIVE_SPECIAL  1
//#define DEBUG_BUS_RECEIVE_FF  1
//#define DEBUG_BUS_RECEIVE_ISO 1
#define DEBUG_COMMAND 1
#define DEBUG_COMMAND_FF 1
#define DEBUG_COMMAND_ISO 1

#define VERSION "001"

#define SERIAL_BPS 115200
#define USE_SERIAL 1
#define USE_BLUETOOTH 1
//#define USE_WIFI 1
//#define USE_SOFTAP 1

//#define USE_LEDS 1

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

// Arduino framework includes ************************************************
#include <WiFi.h>
#include <BluetoothSerial.h>

// Repository included libraries includes, see ./lib/ ************************
#include "ESP32CAN.h"
#include "CAN_config.h"

// Our own includes, see ./include/ ******************************************
#include "config.h"     // Contains WiFi credentials for station mode

// Tidy up defs **************************************************************
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#ifndef DEBUG
#undef DEBUG_BUS_RECEIVE_SPECIAL
#undef DEBUG_BUS_RECEIVE_FF
#undef DEBUG_BUS_RECEIVE_ISO
#undef DEBUG_COMMAND
#undef SERIAL_BPS
#endif

#ifdef USE_LEDS
#define LED_WHITE 26    // ISO-TP frame incoming
#define LED_YELLOW 27   // free fame incoming
#define LED_RED 32      // power
#define LED_GREEN 33    // serial incoming request
#define LED_BLUE 25     // bluetooth connected
#define LED_ON LOW      // active LOW
#define LED_OFF HIGH
#endif

#ifndef USE_WIFI
#undef USE_SOFTAP
#endif

// Bluetooth *****************************************************************
#ifdef USE_BLUETOOTH
BluetoothSerial SerialBT;
#endif

// WiFi *** (sensitive data in config.h, do not include in the repository! ***
#ifdef USE_WIFI
#define MAX_SRV_CLIENTS 1
boolean wiFiIsActive = false;

WiFiServer server(35000);
WiFiClient serverClients[MAX_SRV_CLIENTS];
#endif

// Command *******************************************************************
// structure that defines a textual incoming command
typedef struct
{
  char cmd;
  uint32_t id = 0;
  uint8_t request[8];
  uint8_t requestLength = 0;
  uint8_t reply[8];
  uint8_t replyLength = 0;
} COMMAND;

// CANbus ********************************************************************
CAN_device_t CAN_cfg;

// Free frames storage *******************************************************
// all free frames are stored inside an 2D array storage for all 700 free
// frames is pre-allocated, which will speed up request/response time. Byte
// 8 is length, Byte 9 is reserved for age.
#define SIZE 0x700
uint8_t dataArray[SIZE][10];
int dataArraySize = SIZE;

// ISO-TP message ************************************************************
// structure that defines an ISO-TP message
typedef struct
{
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
} ISO_MESSAGE;

// declare an ISO-TP message
ISO_MESSAGE isoMessage;

// command read buffer *******************************************************
String readBuffer = "";

void setup() {
#ifdef USE_LEDS
  // setup LED's
  pinMode(LED_WHITE, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // they are active low ...
  digitalWrite(LED_WHITE, LED_OFF);
  digitalWrite(LED_YELLOW, LED_OFF);
  digitalWrite(LED_RED, LED_OFF);
  digitalWrite(LED_GREEN, LED_OFF);
  digitalWrite(LED_BLUE, LED_OFF);

  // startup sequence
  digitalWrite(LED_BLUE, LED_ON);
  delay(200);
  digitalWrite(LED_BLUE, LED_OFF);
  digitalWrite(LED_GREEN, LED_ON);
  delay(200);
  digitalWrite(LED_GREEN, LED_OFF);
  digitalWrite(LED_RED, LED_ON);
  delay(200);
  digitalWrite(LED_RED, LED_OFF);
  digitalWrite(LED_YELLOW, LED_ON);
  delay(200);
  digitalWrite(LED_YELLOW, LED_OFF);
  digitalWrite(LED_WHITE, LED_ON);
  delay(200);
  digitalWrite(LED_WHITE, LED_OFF);

  // setup PWM to dim some of the LED's that may stay "always on"
  // like "power" and "bluetooth"
  ledcSetup(0, 5000, 8);
  ledcWrite(0, 250);

  // turn RED (=power) immediately on
  ledcAttachPin(LED_RED, 0);
#endif


#if defined (DEBUG) || defined (USE_SERIAL)
  Serial.begin(SERIAL_BPS);                        // init serial
#endif
#ifdef DEBUG
  Serial.println("");
  Serial.println("");
  Serial.println("CANSee starting ...");
#endif

#ifdef USE_BLUETOOTH
  SerialBT.begin("CANSee");                        // init Bluetooth serial
#endif

#ifdef USE_WIFI
#ifdef USE_SOFTAP
  WiFi.softAP (ssid, password);                    // init WiFi access point
  wiFiIsActive = true;                             // no need to check for active network
#ifdef DEBUG
  IPAddress IP = WiFi.softAPIP ();
  Serial.print ("AP IP address: ");
  Serial.println (IP);
#endif
#else
  WiFi.begin(ssid, password);                      // init WiFi station. Cheking is done in main loop
#endif
  server.begin ();                                 // start the server
#endif

  CAN_cfg.speed = CAN_SPEED_500KBPS;               // init the CAN bus (pins and baudrate)
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  // create a generic RTOS queue for CAN receiving, with 10 positions
  CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
  if (CAN_cfg.rx_queue == 0) {
#ifdef DEBUG
    Serial.println("Can't create CANbus buffer. Stopping");
#endif
    while (1);
  }
  ESP32Can.CANInit();                              // initialize CAN Module

  for (int id = 0; id < dataArraySize; id++) {     // clear the free frame buffer. Data zeroed, length zeroed, not timed out
    for (int i = 0; i < 9; i++) {
      dataArray[id][i] = 0;
    }
    dataArray[id][9] = 0xff;
  }
}


void loop() {

#ifdef USE_LEDS
  if (SerialBT.hasClient())
    ledcAttachPin(LED_BLUE, 0);
  else
  {
    pinMatrixOutDetach(LED_BLUE, false, false);
    // this method is not yet in the actual build!
    //ledcDetachPin(LED_BLUE);
  }
#endif

  // 1. receive next CAN frame from queue
  // removed 3 * portTICK_PERIOD_MS to not block the code
  CAN_frame_t rx_frame;
  if (xQueueReceive (CAN_cfg.rx_queue, &rx_frame, (TickType_t)0) == pdTRUE) {
#ifdef DEBUG_BUS_RECEIVE_SPECIAL
    if (rx_frame.MsgID == 0x5d7) {
      Serial.print (canFrameToString (rx_frame));
    }
#endif
    storeFrame (rx_frame);
  }

  // 2. proceed with input (serial & BT)
  readIncoming ();

  // 3. age data array of free frames
  // to do

  // 4. check WiFi network status if in station mode
#ifdef USE_WIFI
#ifndef USE_SOFTAP
  if (WiFi.status() == WL_CONNECTED) {            // check if connected
    if (!wiFiIsActive) {
#ifdef DEBUG
      IPAddress IP = WiFi.localIP ();
      Serial.print ("IP address: ");
      Serial.println (IP);
#endif
      wiFiIsActive = true;
    }
  } else {
    if (wiFiIsActive) {
      wiFiIsActive = false;
    }
  }
#endif
#endif
}

/*****************************************************************************
 * frame handling function
 * see https://en.wikipedia.org/wiki/ISO_15765-2 for ISO-TP
 */

void storeFrame (CAN_frame_t &frame) {
  if (frame.MsgID < 0x700) {                      // free data stream is < 0x700
#ifdef USE_LEDS
    digitalWrite(LED_YELLOW, LED_ON);
#endif

#ifdef DEBUG_BUS_RECEIVE_FF
    Serial.print ("FF:");
    Serial.print(canFrameToString(frame));
#endif

    for (int i = 0; i < 8; i++)                   // store a copy
      dataArray[frame.MsgID][i] = frame.data.u8[i];
    dataArray[frame.MsgID][8] = frame.FIR.B.DLC;  // and the length
    dataArray[frame.MsgID][9] = 0xff;             // age to ff

#ifdef USE_LEDS
    digitalWrite(LED_YELLOW, LED_OFF);
#endif
  }

  // if there is content and this is the frame we are waiting for
  else if (frame.FIR.B.DLC > 0 && frame.MsgID == isoMessage.id) {
#ifdef USE_LEDS
    digitalWrite(LED_WHITE, LED_ON);
#endif

    uint8_t type = frame.data.u8[0] >> 4;         // id = first nibble

    // single frame answer *****************************************************
    if (type == 0x0) {
#ifdef DEBUG_BUS_RECEIVE_ISO
      Serial.print(">>ISO SING:");
      Serial.print(canFrameToString(frame));
#endif

      uint16_t messageLength = frame.data.u8[0] & 0x0f;// length = second nibble + second byte
      if (messageLength > 7) messageLength = 7;  // this should never happen
      isoMessage.length = messageLength;

      // clear data buffer
      for (int i = 0; i < messageLength; i++)
        isoMessage.data[i] = 0;

      // fill up with this initial first-frame data (should always be 6)
      isoMessage.index = 0;                            // pointer at start of array
      for (int i = 1; i < frame.FIR.B.DLC; i++)        // was starting at 4?
      {
        if (isoMessage.index < isoMessage.length)
        {
          isoMessage.data[isoMessage.index++] = frame.data.u8[i];
        }
      }
      writeOutgoing (isoMessageToString(isoMessage));

      // cancel this message
      isoMessage.id = 0xffff;
    }

    // first frame of a multi-framed message ***********************************
    else if (type == 0x1) {
#ifdef DEBUG_BUS_RECEIVE_ISO
      Serial.print("ISO FRST:");
      Serial.print(canFrameToString(frame));
#endif

      // start by requesting requesing the type Consecutive (0x2) frames by sending a Flow frame
      CAN_frame_t flow;
      flow.FIR.B.FF = CAN_frame_std;                   // set the type to 11 bits
      flow.FIR.B.RTR = CAN_no_RTR;                     // no RTR
      flow.MsgID = isoMessage.requestId;               // send it to the requestId
      flow.FIR.B.DLC = 8;                              // length 8 bytes
      flow.data.u8[0] = 0x30;                          // type Flow (3), flag Clear to send (0)
      flow.data.u8[1] = 0x00;                          // instruct to send all remaining frames without flow control
      flow.data.u8[2] = 0x10;                          // delay between frames <=127 = millis
      flow.data.u8[3] = 0;                             // fill-up
      flow.data.u8[4] = 0;                             // fill-up
      flow.data.u8[5] = 0;                             // fill-up
      flow.data.u8[6] = 0;                             // fill-up
      flow.data.u8[7] = 0;                             // fill-up
      ESP32Can.CANWriteFrame(&flow);

      // build new iso message
      // isoMessage.id = frame.MsgID;                  // .id is already set when sending, and checked when answer was received
      // set final length
      uint16_t messageLength = frame.data.u8[0] & 0x0f;// length = second nibble + second byte
      messageLength <<= 8;
      messageLength |= frame.data.u8[1];
      if (messageLength > 1023) messageLength = 1023;  // this should never happen
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
#ifdef DEBUG_BUS_RECEIVE_ISO
      Serial.print("ISO NEXT:");
      Serial.print(canFrameToString(frame));
#endif

      uint8_t sequence = frame.data.u8[0] & 0x0f;
      if (isoMessage.next == sequence) {
        for (int i = 1; i < frame.FIR.B.DLC; i++) {
          if (isoMessage.index < isoMessage.length) {
            isoMessage.data[isoMessage.index++] = frame.data.u8[i];
          }
        }

        // wait for next message
        if (isoMessage.next++ == 15) isoMessage.next = 0; // was 1, but a rollover should be from 15 to 0

        // is this the last part?
        if (isoMessage.index == isoMessage.length)
        {
          // output the data
          String dataString = isoMessageToString(isoMessage);
#ifdef DEBUG_BUS_RECEIVE_ISO
          Serial.print(">>ISO MSG:");
#endif
          writeOutgoing(dataString);

          // cancel this message
          isoMessage.id = 0xffff;
        }
      } else {
#ifdef DEBUG
        Serial.println("E:ISO Out of sequence, resetting");
#endif
        //writeOutgoing("fff,\n");
        isoMessage.id = 0xffff;
      }
    } else {
#ifdef DEBUG
      Serial.println("E:ISO ignoring unknown frame type:" + String (type));
#endif
      //writeOutgoing("fff,\n");
    }

#ifdef USE_LEDS
    digitalWrite(LED_WHITE, LED_OFF);
#endif
  } else {
#ifdef DEBUG
    Serial.println("E:ISO frame of unrequested id:" + String(frame.MsgID, HEX));
#endif
    //writeOutgoing("fff,\n");
  }
}

/*****************************************************************************
 * I/O functions
 */

void writeOutgoing (String o) {
  writeOutgoingSerial (o);
  writeOutgoingBluetooth (o);
  writeOutgoingWiFi (o);
}

void writeOutgoingSerial (String o) {
#if defined (DEBUG) || defined (USE_SERIAL)
  Serial.print (o);
#endif
}

void writeOutgoingBluetooth (String o) {
#ifdef USE_BLUETOOTH
  SerialBT.print (o);
#endif
}

void writeOutgoingWiFi (String o) {
#ifdef USE_WIFI
  if (!wiFiIsActive) return;
  char buf[1024];
  unsigned int len;
  o.replace ("\n", "\n\r");
  if ((len = o.length()) > 1024) len = 1024;
  o.toCharArray(buf, len);
  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      //serverClients[i].write(o);
      serverClients[i].write(buf, len);
      //delay(1);
    }
  }
#endif
}

void readIncoming() {
  readIncomingSerial();
  readIncomingBluetooth();
  readIncomingWiFi();
}

void readIncomingSerial() {
#if defined (DEBUG) || defined (USE_SERIAL)
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (readBuffer != "") {
        processCommand(readBuffer);
        readBuffer = "";
      }
    } else {
      readBuffer += ch;
    }
  }
#endif
}

void readIncomingBluetooth() {
  if (SerialBT.available()) {
    char ch = SerialBT.read();
    if (ch == '\n' || ch == '\r') {
      if (readBuffer != "") {
        processCommand(readBuffer);
        readBuffer = "";
      }
    } else {
      readBuffer += ch;
    }
  }
}

void readIncomingWiFi() {
#ifdef USE_WIFI
  if (!wiFiIsActive) return;
  // no need to check for WL_CONNECTED, as we are the access point
  uint8_t i;
  if (server.hasClient()) {                        // check if there are any *NEW* clients
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {        //if so, find free or disconnected spot
      if (!serverClients[i] || !serverClients[i].connected()) {
        if (serverClients[i]) {                    // if not free (so disconnected)
          serverClients[i].stop();                 // stop the client
#ifdef DEBUG_COMMAND
          Serial.print("Disconnected: ");
          Serial.println(i);
#endif
        }
        serverClients[i] = server.available();     // fetch the client
        if (serverClients[i]) {                    // it should be here
#ifdef DEBUG_COMMAND
          Serial.print("New client: ");
          Serial.print(i); Serial.print(' ');
          Serial.println(serverClients[i].remoteIP());
#endif
        } else {                                   // if gone, oh well
#ifdef DEBUG_COMMAND
          Serial.println("available broken");
#endif
        }
        break;
      }
    }
    if (i >= MAX_SRV_CLIENTS) {                    //no free/disconnected spot so reject

      server.available().stop();
#ifdef DEBUG_COMMAND
      Serial.println("Refused");
#endif
    }
  }

  for (i = 0; i < MAX_SRV_CLIENTS; i++) {          // check clients for data
    if (serverClients[i] && serverClients[i].connected()) {
      while (serverClients[i].available()) {       // if there is data
        char ch = serverClients[i].read();         // get it
        if (ch == '\n' || ch == '\r') {            // buffer / process it
          if (readBuffer != "") {
            processCommand(readBuffer);
            readBuffer = "";
          }
        } else {
          readBuffer += ch;
        }
      }
    } else {                                       // no client, or unconnected
      if (serverClients[i]) {                      // if there is a client (so unconnected)
        serverClients[i].stop();                   // stop the client
#ifdef DEBUG_COMMAND
        Serial.print("Disconnected: ");
        Serial.println(i);
#endif
      }
    }
  }
#endif
}

// execute a command
void processCommand(String & line) {
#ifdef DEBUG_COMMAND
  Serial.println("< com:" + line);
#endif
  COMMAND command = decodeCommand(line);

  // output all buffered frames **********************************************
  if (command.cmd == 'a') {
    int count = 0;
    for (int id = 0; id < dataArraySize; id++) {
      if (dataArray[id][9]) { // print length 0 frames, but do not print timeout frames
        writeOutgoing (bufferedFrameToString (id)); // includes \n
        count++;
      }
    }
#ifdef DEBUG_COMMAND
    Serial.println("> fcn:" + String (count));
#endif
    return;
  }

  // get a frame *************************************************************
  else if (command.cmd == 'g') {
    if (command.id < dataArraySize) {
      writeOutgoing (bufferedFrameToString(command.id));
    } else {
#ifdef DEBUG_COMMAND
      Serial.print ("> com:ID out of bounds (0 - 0x6ff)");
#endif
      writeOutgoing (bufferedFrameToString(0));
    }
    return;
  }

  // request an ISO-TP frame *************************************************
  else if (command.cmd == 'i') {
    // only accept this command if the requested ID belongs to an ISO-TP frame
    if (command.id < 0x700 || command.id > 0x7ff) {
#ifdef DEBUG
      Serial.println ("E:ID out of bounds (0x700 - 0x7ff)");
#endif
      writeOutgoing (String (command.id, HEX) + "\n");
      return;
    }
    // store ID
    isoMessage.id = command.id;                    // expected ID of answer
    if ((isoMessage.requestId = getRequestId(command.id)) == 0) { // ID to send request to
#ifdef DEBUG_COMMAND
      Serial.println ("> com:" + String (command.id, HEX) + " has no corresponding request ID");
#endif
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
#ifdef DEBUG_COMMAND_ISO
    Serial.print ("> com:Sending ISOTP request:");
    Serial.print (canFrameToString (frame));
#endif
    ESP32Can.CANWriteFrame (&frame);
        // --> any incoming frames with the given id will be handled by "storeFrame" and send off if complete
    return;
  }

  // inject a frame via serial / BT input ************************************
  else if (command.cmd == 't') {
    CAN_frame_t frame;
    frame.MsgID = command.id;
    frame.FIR.B.DLC = command.requestLength;
    for (int i = 0; i < command.requestLength; i++)
      frame.data.u8[i] = command.request[i];
#ifdef DEBUG_COMMAND
    Serial.print ("> com:Injecting " + canFrameToString (frame));
#endif
    storeFrame(frame);
    // storeframe will output if free frame or ISO-TP Single
    // writeOutgoing (String (command.id, HEX) + "\n");
    return;
  }

  // filter (deprecated) *****************************************************
  else if (command.cmd == 'f')
  {
#ifdef DEBUG_COMMAND
    Serial.println ("> com:Filter " + String (command.id, HEX));
#endif
    writeOutgoing (String (command.id, HEX) + "\n");
    return;
  }


  // give up *****************************************************************
  else {
#ifdef DEBUG
    Serial.println ("> com:Unknown command " + String (command.cmd));
#endif
    writeOutgoing("fff,\n");
    return;
  }
}

// parse a string into a command
COMMAND decodeCommand(String &input)
{
  COMMAND result;

  // clear out older data
  result.id = 0;
  result.requestLength = 0;
  result.replyLength = 0;

  // trim whitespaces
  input.trim();

  // stop if input is empty
  if (input.length() == 0) return result;

  // the first letter is the command
  result.cmd = input.charAt(0);
  input.remove(0, 1);

  // if there is something more,
  if (input.length() != 0)
  {
    // get the ID
    char ch;
    String id = "";
    do
    {
      ch = input.charAt(0);
      if (ch != ',') id += ch;
      input.remove(0, 1);
    }
    while (input.length() != 0 && ch != ',');
    result.id = hexToDec(id);
  }

  // if there is something more,
  if (input.length() != 0)
  {
    // get the REQUEST
    char ch;
    String request = "";
    do
    {
      ch = input.charAt(0);
      if (ch != ',') request += ch;
      input.remove(0, 1);
    }
    while (input.length() != 0 && ch != ',');
    for (int i = 0; i < request.length() && result.requestLength < 8; i += 2) // check for overflow
    {
      result.request[result.requestLength] = hexToDec(request.substring(i, i + 2));
      result.requestLength++;
    }
  }

  // if there is something more,
  if (input.length() != 0)
  {
    // get the REPLY
    char ch;
    String reply = "";
    do
    {
      ch = input.charAt(0);
      if (ch != ',') reply += ch;
      input.remove(0, 1);
    }
    while (input.length() != 0 && ch != ',');
    for (int i = 0; i < reply.length() && result.replyLength < 8; i += 2) // check for overflow
    {
      result.reply[result.replyLength] = hexToDec(reply.substring(i, i + 2));
      result.replyLength++;
    }
  }
  return result;
}

/*****************************************************************************
 * Converter functions
*/

// convert a CAN_frame to readable hex output format
String canFrameToString(CAN_frame_t &frame)
{
  String dataString = String(frame.MsgID, HEX) + ",";
  for (int i = 0; i < frame.FIR.B.DLC; i++)
  {
    dataString += getHex(frame.data.u8[i]);
  }
  dataString += "\n";
  return dataString;
}

// convert a ISO-TP message to readable hex output format
String isoMessageToString(ISO_MESSAGE & message)
{
  String dataString = String(message.id, HEX) + ",";
  for (int i = 0; i < message.length; i++)
  {
    dataString += getHex(message.data[i]);
  }
  dataString += "\n";
  return dataString;
}

// convert a buffered frame to readable hex output format
String bufferedFrameToString (int id)
{
  uint8_t *fd = dataArray[id];
  String dataString = String (id, HEX) + ",";
  if (fd[9]) {
    for (int i = 0; i < fd[8]; i++)
    {
      dataString += getHex(fd[i]);
    }
  }
  dataString += "\n";
  return dataString;
}

/*****************************************************************************
 * Utility functions
 */

String getHexSimple(uint8_t num)
{
  String stringOne =  String(num, HEX);
  return stringOne;
}

String getHex(uint8_t num)
{
  String stringOne =  String(num, HEX);
  if (stringOne.length() < 2) stringOne = "0" + stringOne;
  return stringOne;
}

unsigned int hexToDec(String hexString) {

  unsigned int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++) {
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57)
      nextInt = map(nextInt, 48, 57, 0, 9);
    else if (nextInt >= 65 && nextInt <= 70)
      nextInt = map(nextInt, 65, 70, 10, 15);
    else if (nextInt >= 97 && nextInt <= 102)
      nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    decValue = (decValue * 16) + nextInt;
  }
  return decValue;
}

/*****************************************************************************
 * ZOE CAN computer related functions
 */
uint16_t getRequestId(uint16_t responseId)
{                     // from ECU id   to ECU id
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
