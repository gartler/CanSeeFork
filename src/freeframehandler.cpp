#include "freeframehandler.h"


static FREEFRAME_t freeframes[FREEFRAMEARRAYSIZE];

void freeframe_init () {
  for (int id = 0; id < FREEFRAMEARRAYSIZE; id++) {     // clear the free frame buffer. Data zeroed, length zeroed, not timed out
    for (int i = 0; i < 8; i++) freeframes[id].data[i] = 0;
    freeframes[id].length = 0;
    freeframes[id].age = 0xff;
  }
}

void storeFreeframe (CAN_frame_t &frame) {
  if (frame.MsgID < FREEFRAMEARRAYSIZE) {
    for (int i = 0; i < 8; i++) {                   // store a copy
      freeframes[frame.MsgID].data[i] = frame.data.u8[i];
    }
    freeframes[frame.MsgID].length = frame.FIR.B.DLC; // and the length
    freeframes[frame.MsgID].age = 0xff;            // age to ff
  }
}

FREEFRAME_t *getFreeframe (uint32_t id) {
  return &freeframes [id];
}

// convert a buffered frame to readable hex output format
String bufferedFrameToString (uint32_t id) {
  String dataString = String (id, HEX) + ",";
  if (freeframes[id].age) {
    for (int i = 0; i < freeframes[id].length; i++) {
      dataString += getHex(freeframes[id].data[i]);
    }
  }
  dataString += "\n";
  return dataString;
}
