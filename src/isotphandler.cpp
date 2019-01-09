#include "isotphandler.h"

static CS_CONFIG_t *isotp_config;
static void (*isotp_process)(String o);
static ISO_MESSAGE_t isoMessage;                   // declare an ISO-TP message

#define DEBUG_COMMAND_ISO     0x10

void isotp_init (CS_CONFIG_t *config, void (*p)(String o)) {
  isotp_config = config;
  isotp_process = p;
}

void storeIsotpframe (CAN_frame_t &frame, uint8_t bus) {
  // if there is content and this is the frame we are waiting for
  if (frame.FIR.B.DLC > 0 && frame.MsgID == isoMessage.id) {

    uint8_t type = frame.data.u8[0] >> 4;          // id = first nibble

    // single frame answer ***************************************************
    if (type == 0x0) {
      if (isotp_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) {
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
      isotp_process (isoMessageToString (isoMessage));

      // cancel this message
      isoMessage.id = 0xffff;
    }

    // first frame of a multi-framed message *********************************
    else if (type == 0x1) {
      if (isotp_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) {
        Serial.print("ISO FRST:");
        Serial.print(canFrameToString(frame));
      }

      // start by requesting requesing the type Consecutive (0x2) frames by sending a Flow frame
      can_send_flow (isoMessage.requestId, bus);

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
      if (isotp_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) {
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
          if (isotp_config->mode_debug & DEBUG_BUS_RECEIVE_ISO) Serial.print(">>ISO MSG:");
          isotp_process (dataString);

          // cancel this message
          isoMessage.id = 0xffff;
        }
      } else {
        if (isotp_config->mode_debug) Serial.println("E:ISO Out of sequence, resetting");
        //writeOutgoing("fff,\n");
        isoMessage.id = 0xffff;
      }
    } else {
      if (isotp_config->mode_debug) Serial.println("E:ISO ignoring unknown frame type:" + String (type));
    }

  } else {
    if (isotp_config->mode_debug) Serial.println("E:ISO frame of unrequested id:" + String(frame.MsgID, HEX));
  }
}

void can_send_flow (uint16_t requestId, uint8_t bus) {
  CAN_frame_t flow;
  flow.FIR.B.FF = CAN_frame_std;                   // set the type to 11 bits
  flow.FIR.B.RTR = CAN_no_RTR;                     // no RTR
  flow.MsgID = requestId;                          // send it to the requestId
  flow.FIR.B.DLC = 8;                              // length 8 bytes
  flow.data.u8[0] = 0x30;                          // type Flow (3), flag Clear to send (0)
  flow.data.u8[1] = 0x00;                          // instruct to send all remaining frames without flow control
  flow.data.u8[2] = 0x00;                          // delay between frames <=127 = millis, can maybe set to 0
  flow.data.u8[3] = 0;                             // fill-up
  flow.data.u8[4] = 0;                             // fill-up
  flow.data.u8[5] = 0;                             // fill-up
  flow.data.u8[6] = 0;                             // fill-up
  flow.data.u8[7] = 0;                             // fill-up
  can_send (&flow, bus);
}

void request (uint32_t id, int16_t length, uint8_t *request, uint8_t bus) {
  // only accept this command if the requested ID belongs to an ISO-TP frame
  if (id < 0x700 || id > 0x7ff) {
    if (isotp_config->mode_debug) Serial.println ("E:ID out of bounds (0x700 - 0x7ff)");
    isotp_process (String (id, HEX) + "\n");
    return;
  }
  // store ID
  isoMessage.id = id;                              // expected ID of answer
  if ((isoMessage.requestId = getRequestId (id)) == 0) { // ID to send request to
    if (isotp_config->mode_debug & DEBUG_COMMAND) Serial.println ("> com:" + String (id, HEX) + " has no corresponding request ID");
    isotp_process (String (id, HEX) + "\n");
    return;
  }
  // store request
  isoMessage.requestLength = length;
  if (isoMessage.requestLength > 7) isoMessage.requestLength = 7; // this should never happen (yet)
  for (int i = 0; i < length; i++) {
    isoMessage.request[i] = request[i];
  }
  CAN_frame_t frame;                             // build the CAN frame
  frame.FIR.B.FF = CAN_frame_std;                // set the type to 11 bits
  frame.FIR.B.RTR = CAN_no_RTR;                  // no RTR
  frame.MsgID = isoMessage.requestId;            // set the ID
  frame.FIR.B.DLC = 8; //command.requestLength + 1; // set the length. Note some ECU's like DLC 8
  for (int i = 0; i < frame.FIR.B.DLC; i++) {      // zero out frame
    frame.data.u8[i] = 0;
  }

  // we are assuming here that requests are always single frame. This is formally not true, but good enough for us
  frame.data.u8[0] = (length & 0x0f);

  for (int i = 0; i < length; i++) {                // fill up the other bytes with the request
    frame.data.u8[i + 1] = isoMessage.request[i];
  }

  // send the frame
  if (isotp_config->mode_debug & DEBUG_COMMAND_ISO) {
    Serial.print ("> com:Sending ISOTP request:");
    Serial.print (canFrameToString (frame));
  }
  can_send (&frame, bus);
  // --> any incoming frames with the given id will be handled by "storeFrame" and send off if complete

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
