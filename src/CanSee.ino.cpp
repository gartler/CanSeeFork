# 1 "/tmp/tmpZoodxm"
#include <Arduino.h>
# 1 "/home/jeroen/Documents/PlatformIO/Projects/CanSee/src/CanSee.ino"
# 21 "/home/jeroen/Documents/PlatformIO/Projects/CanSee/src/CanSee.ino"
#define DEBUG_BUS_RECEIVE_FF 0x01
#define DEBUG_BUS_RECEIVE_ISO 0x02
#define DEBUG_COMMAND 0x04
#define DEBUG_COMMAND_FF 0x08
#define DEBUG_COMMAND_ISO 0x10

#define VERSION "001"

#define SERIAL_BPS 115200
# 48 "/home/jeroen/Documents/PlatformIO/Projects/CanSee/src/CanSee.ino"
#include <WiFi.h>
#include <BluetoothSerial.h>


#include "config.h"
#include "canhandler.h"
#include "leds.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


CS_CONFIG *cansee_config;


BluetoothSerial SerialBT;


#define MAX_SRV_CLIENTS 1
boolean wiFiIsActive = false;

WiFiServer server(35000);
WiFiClient serverClients[MAX_SRV_CLIENTS];



typedef struct {
  char cmd;
  uint32_t id = 0;
  uint8_t request[8];
  uint8_t requestLength = 0;
  uint8_t reply[32];
  uint8_t replyLength = 0;
  char line[48];
} COMMAND;


CAN_device_t CAN_cfg;





#define SIZE 0x700
uint8_t dataArray[SIZE][10];
int dataArraySize = SIZE;


typedef struct {
  uint16_t id = 0xffff;
  uint16_t requestId = 0xffff;
  uint16_t length = 0;
  uint8_t index = 0;
  uint8_t next = 1;
  uint8_t request[8];
  uint8_t requestLength = 0;



  uint8_t data[1024];
} ISO_MESSAGE;


ISO_MESSAGE isoMessage;


String readBuffer = "";
void setup();
void loop();
void storeFrame (CAN_frame_t &frame);
void writeOutgoing (String o);
void writeOutgoingSerial (String o);
void writeOutgoingBluetooth (String o);
void writeOutgoingWiFi (String o);
void readIncoming();
void readIncomingSerial();
void readIncomingBluetooth();
void readIncomingWiFi();
void processCommand(String &line);
COMMAND decodeCommand (String &input);
String canFrameToString(CAN_frame_t &frame);
String isoMessageToString(ISO_MESSAGE & message);
String bufferedFrameToString (int id);
String getHexSimple(uint8_t num);
String getHex(uint8_t num);
unsigned int hexToDec(String hexString);
uint16_t getRequestId(uint16_t responseId);
#line 119 "/home/jeroen/Documents/PlatformIO/Projects/CanSee/src/CanSee.ino"
void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println ("");
  Serial.println ("");
  Serial.println ("CANSee starting...");

  delay (500);
  pinMode (0, INPUT);
  if (!digitalRead (0)) {
    Serial.println("Reset config...");
    setConfigToEeprom (true);
  }
  cansee_config = getConfigFromEeprom ();

  if (cansee_config->mode_debug) {
    Serial.print ("Version:   "); Serial.println (VERSION);
    Serial.println ("Serial:    " + String (cansee_config->mode_serial , HEX));
    Serial.println ("Bluetooth: " + String (cansee_config->mode_bluetooth, HEX));
    Serial.println ("WiFi:      " + String (cansee_config->mode_wifi , HEX));
    Serial.println ("Leds:      " + String (cansee_config->mode_leds , HEX));
    Serial.println ("Debug:     " + String (cansee_config->mode_debug , HEX));
  }
  if (!cansee_config->mode_serial && !cansee_config->mode_debug) Serial.end();

  leds_init (cansee_config);

  if (cansee_config->mode_bluetooth) {
    if (cansee_config->mode_debug) Serial.println("Bluetooth " + String (cansee_config->name_bluetooth) + " starting ...");
    SerialBT.begin(cansee_config->name_bluetooth);
  }

  if (cansee_config->mode_wifi == WIFI_SOFTAP) {
    if (cansee_config->mode_debug) Serial.println("Wifi AP " + String (cansee_config->ssid_ap) + " starting ...");
    WiFi.softAP (cansee_config->ssid_ap, cansee_config->password_ap);
    wiFiIsActive = true;
    if (cansee_config->mode_debug) {
      IPAddress IP = WiFi.softAPIP ();
      Serial.print ("AP IP address: ");
      Serial.println (IP);
    }
    server.begin ();
  } else if (cansee_config->mode_wifi == WIFI_STATION) {
    if (cansee_config->mode_debug) Serial.println("Wifi ST " + String (cansee_config->ssid_station) + " starting ...");
    WiFi.begin(cansee_config->ssid_station, cansee_config->password_station);
    server.begin ();
  }

  can_init (cansee_config);

  for (int id = 0; id < dataArraySize; id++) {
    for (int i = 0; i < 9; i++) dataArray[id][i] = 0;
    dataArray[id][9] = 0xff;
  }
}


void loop() {


  CAN_frame_t rx_frame;
  if (xQueueReceive (CAN_cfg.rx_queue, &rx_frame, (TickType_t)0) == pdTRUE) {
    storeFrame (rx_frame);
  }


  led_set (LED_BLUE, SerialBT.hasClient());
  readIncoming ();





  if (cansee_config->mode_wifi == WIFI_STATION) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!wiFiIsActive) {
        if (cansee_config->mode_debug) {
          IPAddress IP = WiFi.localIP ();
          Serial.print ("IP address: ");
          Serial.println (IP);
        }
        wiFiIsActive = true;
      }
    } else {
      if (wiFiIsActive) {
        wiFiIsActive = false;
      }
    }
  }
}






void storeFrame (CAN_frame_t &frame) {
  if (frame.MsgID < 0x700) {
    led_set (LED_YELLOW, true);

    if (cansee_config->mode_debug & DEBUG_BUS_RECEIVE_FF) {
      Serial.print ("FF:");
      Serial.print(canFrameToString(frame));
    }

    for (int i = 0; i < 8; i++)
    dataArray[frame.MsgID][i] = frame.data.u8[i];
    dataArray[frame.MsgID][8] = frame.FIR.B.DLC;
    dataArray[frame.MsgID][9] = 0xff;

    led_set (LED_YELLOW, false);
  }


  else if (frame.FIR.B.DLC > 0 && frame.MsgID == isoMessage.id) {
    led_set (LED_WHITE, true);

    uint8_t type = frame.data.u8[0] >> 4;


    if (type == 0x0) {
      if (cansee_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) {
        Serial.print(">>ISO SING:");
        Serial.print(canFrameToString(frame));
      }

      uint16_t messageLength = frame.data.u8[0] & 0x0f;
      if (messageLength > 7) messageLength = 7;
      isoMessage.length = messageLength;


      for (int i = 0; i < messageLength; i++)
      isoMessage.data[i] = 0;


      isoMessage.index = 0;
      for (int i = 1; i < frame.FIR.B.DLC; i++) {
        if (isoMessage.index < isoMessage.length) {
          isoMessage.data[isoMessage.index++] = frame.data.u8[i];
        }
      }
      writeOutgoing (isoMessageToString (isoMessage));


      isoMessage.id = 0xffff;
    }


    else if (type == 0x1) {
      if (cansee_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) {
        Serial.print("ISO FRST:");
        Serial.print(canFrameToString(frame));
      }


      can_send_flow (isoMessage.requestId);




      uint16_t messageLength = frame.data.u8[0] & 0x0f;
      messageLength <<= 8;
      messageLength |= frame.data.u8[1];
      if (messageLength > 1023) messageLength = 1023;
      isoMessage.length = messageLength;


      for (int i = 0; i < messageLength; i++)
      isoMessage.data[i] = 0;


      isoMessage.next = 1;


      isoMessage.index = 0;
      for (int i = 2; i < frame.FIR.B.DLC; i++) {
        if (isoMessage.index < isoMessage.length) {
          isoMessage.data[isoMessage.index++] = frame.data.u8[i];
        }
      }
    }


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


        if (isoMessage.next++ == 15) isoMessage.next = 0;


        if (isoMessage.index == isoMessage.length) {

          String dataString = isoMessageToString(isoMessage);
          if (cansee_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) Serial.print(">>ISO MSG:");
          writeOutgoing(dataString);


          isoMessage.id = 0xffff;
        }
      } else {
        if (cansee_config->mode_debug) Serial.println("E:ISO Out of sequence, resetting");

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



void writeOutgoing (String o) {
  writeOutgoingSerial (o);
  writeOutgoingBluetooth (o);
  writeOutgoingWiFi (o);
}

void writeOutgoingSerial (String o) {
  if (cansee_config->mode_serial || cansee_config->mode_debug) Serial.print (o);
}

void writeOutgoingBluetooth (String o) {
  if (cansee_config->mode_bluetooth) SerialBT.print (o);
}

void writeOutgoingWiFi (String o) {
  if (!cansee_config->mode_wifi) return;
  if (!wiFiIsActive) return;
  char buf[1024];
  unsigned int len;

  if ((len = o.length()) > 1024) len = 1024;
  o.toCharArray(buf, len);
  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {

      serverClients[i].write(buf, len);

    }
  }
}

void readIncoming() {
  readIncomingSerial();
  readIncomingBluetooth();
  readIncomingWiFi();
}

void readIncomingSerial() {
  if (!cansee_config->mode_serial && !cansee_config->mode_debug) return;
  if (!Serial.available()) return;
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

void readIncomingBluetooth() {
  if (!cansee_config->mode_bluetooth) return;
  if (!SerialBT.available()) return;
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

void readIncomingWiFi() {
  if (!cansee_config->mode_wifi) return;
    if (!wiFiIsActive) return;

    uint8_t i;
    if (server.hasClient()) {
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (!serverClients[i] || !serverClients[i].connected()) {
          if (serverClients[i]) {
            serverClients[i].stop();
            if (cansee_config->mode_debug & DEBUG_COMMAND) {
              Serial.print("Disconnected: ");
              Serial.println(i);
            }
          }
          serverClients[i] = server.available();
          if (serverClients[i]) {
            if (cansee_config->mode_debug & DEBUG_COMMAND) {
              Serial.print("New client: ");
              Serial.print(i); Serial.print(' ');
              Serial.println(serverClients[i].remoteIP());
            }
          } else {
            if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println("available broken");
          }
          break;
        }
      }
      if (i >= MAX_SRV_CLIENTS) {

        server.available().stop();
        if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println("Refused");
      }
    }

    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i] && serverClients[i].connected()) {
        while (serverClients[i].available()) {
          char ch = serverClients[i].read();
          if (ch == '\n' || ch == '\r') {
            if (readBuffer != "") {
              processCommand(readBuffer);
              readBuffer = "";
            }
          } else {
            readBuffer += ch;
          }
        }
      } else {
        if (serverClients[i]) {
          serverClients[i].stop();
          if (cansee_config->mode_debug & DEBUG_COMMAND) {
            Serial.print("Disconnected: ");
            Serial.println(i);
          }
        }
      }
  }
}


void processCommand(String &line) {
  uint8_t bus;

  if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println("< com:" + line);
  COMMAND command = decodeCommand(line);

  bus = (command.id & 0x40000000) ? 1 : 0;
  command.id &= 0x1fffffff;

  switch (command.cmd) {


    case 'a': {
      int count = 0;
      for (int id = 0; id < dataArraySize; id++) {
        if (dataArray[id][9]) {
          writeOutgoing (bufferedFrameToString (id));
          count++;
        }
      }
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println("> fcn:" + String (count));
    }
    break;


    case 'g':
    if (command.id < dataArraySize) {
      writeOutgoing (bufferedFrameToString(command.id));
    } else {
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.print ("> com:ID out of bounds (0 - 0x6ff)");
      writeOutgoing (bufferedFrameToString(0));
    }
    break;


    case 'i':

    if (command.id < 0x700 || command.id > 0x7ff) {
      if (cansee_config->mode_debug) Serial.println ("E:ID out of bounds (0x700 - 0x7ff)");
      writeOutgoing (String (command.id, HEX) + "\n");
      return;
    }

    isoMessage.id = command.id;
    if ((isoMessage.requestId = getRequestId(command.id)) == 0) {
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println ("> com:" + String (command.id, HEX) + " has no corresponding request ID");
      writeOutgoing (String (command.id, HEX) + "\n");
      return;
    }

    isoMessage.requestLength = command.requestLength;
    if (isoMessage.requestLength > 7) isoMessage.requestLength = 7;
    for (int i = 0; i < command.requestLength; i++)
    isoMessage.request[i] = command.request[i];

    CAN_frame_t frame;
    frame.FIR.B.FF = CAN_frame_std;
    frame.FIR.B.RTR = CAN_no_RTR;
    frame.MsgID = isoMessage.requestId;
    frame.FIR.B.DLC = 8;
    for (int i = 0; i < frame.FIR.B.DLC; i++)
    frame.data.u8[i] = 0;


    frame.data.u8[0] = (command.requestLength & 0x0f);

    for (int i = 0; i < command.requestLength; i++)
    frame.data.u8[i + 1] = command.request[i];


    if (cansee_config->mode_debug & DEBUG_COMMAND_ISO) {
      Serial.print ("> com:Sending ISOTP request:");
      Serial.print (canFrameToString (frame));
    }
    can_send (&frame, bus);

    break;


    case 't': {
      CAN_frame_t frame;
      frame.MsgID = command.id;
      frame.FIR.B.DLC = command.requestLength;
      for (int i = 0; i < command.requestLength; i++)
      frame.data.u8[i] = command.request[i];
      if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.print ("> com:Injecting " + canFrameToString (frame));
      storeFrame(frame);


    }
    break;


    case 'f':
    if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println ("> com:Filter " + String (command.id, HEX));
    writeOutgoing (String (command.id, HEX) + "\n");
    break;


    case 'n':
    if (cansee_config->mode_debug & DEBUG_COMMAND) Serial.println ("> com:config " + String (command.id, HEX));
    switch (command.id) {
      case 0x100:
      cansee_config->mode_serial = command.request [0];
      cansee_config->mode_bluetooth = command.request [1];
      cansee_config->mode_wifi = command.request [2];
      cansee_config->mode_leds = command.request [3];
      cansee_config->mode_debug = command.request [4];
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
      case 0x500:
      cansee_config->can1_speed = command.request [0];
      cansee_config->can1_rx = command.request [1];
      cansee_config->can1_tx = command.request [2];
      case 0x501:
      cansee_config->can2_speed = command.request [0];
      cansee_config->can2_rx = command.request [1];
      cansee_config->can2_tx = command.request [2];
      break;
    }
    setConfigToEeprom (false);
    writeOutgoing (String (command.id, HEX) + "\n");
    break;


    case 'z':
    ESP.restart();
    break;


    default:
    if (cansee_config->mode_debug) Serial.println ("> com:Unknown command " + String (command.cmd));
    writeOutgoing("fff,\n");
    break;
  }
}


COMMAND decodeCommand (String &input) {
  COMMAND result;
  result.id = 0;
  result.requestLength = 0;
  result.replyLength = 0;

  input.trim();
  strncpy (result.line, input.c_str(), sizeof (result.line));

  if (input.length() == 0) return result;

  result.cmd = input.charAt(0);
  input.remove(0, 1);

  if (input.length() != 0) {
    char ch;
    String id = "";
    do {
      ch = input.charAt(0);
      if (ch != ',') id += ch;
      input.remove(0, 1);
    } while (input.length() != 0 && ch != ',');
    result.id = hexToDec(id);
  }

  if (input.length() != 0) {
    char ch;
    String request = "";
    do {
      ch = input.charAt(0);
      if (ch != ',') request += ch;
      input.remove(0, 1);
    } while (input.length() != 0 && ch != ',');
    for (int i = 0; i < request.length() && result.requestLength < 8; i += 2) {
      result.request[result.requestLength] = hexToDec(request.substring(i, i + 2));
      result.requestLength++;
    }
  }

  if (input.length() != 0) {
    char ch;
    String reply = "";
    do {
      ch = input.charAt(0);
      if (ch != ',') reply += ch;
      input.remove(0, 1);
    } while (input.length() != 0 && ch != ',');
    for (int i = 0; i < reply.length() && result.replyLength < 8; i += 2) {
      result.reply[result.replyLength] = hexToDec(reply.substring(i, i + 2));
      result.replyLength++;
    }
  }
  return result;
}




String canFrameToString(CAN_frame_t &frame) {
  String dataString = String(frame.MsgID, HEX) + ",";
  for (int i = 0; i < frame.FIR.B.DLC; i++) {
    dataString += getHex(frame.data.u8[i]);
  }
  dataString += "\n";
  return dataString;
}


String isoMessageToString(ISO_MESSAGE & message) {
  String dataString = String(message.id, HEX) + ",";
  for (int i = 0; i < message.length; i++) {
    dataString += getHex(message.data[i]);
  }
  dataString += "\n";
  return dataString;
}


String bufferedFrameToString (int id) {
  uint8_t *fd = dataArray[id];
  String dataString = String (id, HEX) + ",";
  if (fd[9]) {
    for (int i = 0; i < fd[8]; i++) {
      dataString += getHex(fd[i]);
    }
  }
  dataString += "\n";
  return dataString;
}


String getHexSimple(uint8_t num) {
  String stringOne = String(num, HEX);
  return stringOne;
}

String getHex(uint8_t num) {
  String stringOne = String(num, HEX);
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


uint16_t getRequestId(uint16_t responseId) {

  if (responseId == 0x7ec) return 0x7e4;
  else if (responseId == 0x7da) return 0x7ca;
  else if (responseId == 0x7bb) return 0x79b;
  else if (responseId == 0x77e) return 0x75a;
  else if (responseId == 0x772) return 0x752;
  else if (responseId == 0x76d) return 0x74d;
  else if (responseId == 0x763) return 0x743;
  else if (responseId == 0x762) return 0x742;
  else if (responseId == 0x760) return 0x740;
  else if (responseId == 0x7bc) return 0x79c;
  else if (responseId == 0x765) return 0x745;
  else if (responseId == 0x764) return 0x744;
  else if (responseId == 0x76e) return 0x74e;
  else if (responseId == 0x793) return 0x792;
  else if (responseId == 0x7b6) return 0x796;
  else if (responseId == 0x722) return 0x702;
  else if (responseId == 0x767) return 0x747;
  else return 0;
}