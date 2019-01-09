#include "freeframehandler.h"

static CS_CONFIG_t *freeframe_config;
static void (*freeframe_process)(String o);
static FREEFRAME_t freeframes[FREEFRAMEARRAYSIZE];

void freeframe_init (CS_CONFIG_t *config, void (*p)(String o)) {
  freeframe_config = config;
  freeframe_process = p;
  for (int id = 0; id < FREEFRAMEARRAYSIZE; id++) {     // clear the free frame buffer. Data zeroed, length zeroed, not timed out
    for (int i = 0; i < 8; i++) freeframes[id].data[i] = 0;
    freeframes[id].length = 0;
    freeframes[id].age = 0xff;
  }
}

void storeFreeframe (CAN_frame_t &frame, uint8_t bus) {
  if (frame.MsgID < FREEFRAMEARRAYSIZE) {
    //freeframe_process (String (frame.MsgID, HEX) + "\n");
    return;
  }
  if (freeframe_config->mode_debug & DEBUG_BUS_RECEIVE_FF) {
    Serial.print ("FF:");
    Serial.print(canFrameToString(frame));
  }
  for (int i = 0; i < 8; i++) {                   // store a copy
    freeframes[frame.MsgID].data[i] = frame.data.u8[i];
  }
  freeframes[frame.MsgID].length = frame.FIR.B.DLC; // and the length
  freeframes[frame.MsgID].age = 0xff;            // age to ff
}

FREEFRAME_t *getFreeframe (uint32_t id, uint8_t bus) {
  if (id >= FREEFRAMEARRAYSIZE) return &freeframes [0];
  return &freeframes [id];
}

void requestFreeframe  (uint32_t id, uint8_t bus) {
  freeframe_process (bufferedFrameToString (id, bus));
}

// convert a buffered frame to readable hex output format
String bufferedFrameToString (uint32_t id, uint8_t bus) {
  String dataString = String (id, HEX) + ",";
  if (freeframes[id].age) {
    for (int i = 0; i < freeframes[id].length; i++) {
      dataString += getHex(freeframes[id].data[i]);
    }
  }
  dataString += "\n";
  return dataString;
}
