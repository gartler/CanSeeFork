#include "canhandler.h"

static CS_CONFIG_t *can_config;
static uint8_t used_bus = 1;
CAN_device_t CAN_cfg;

void can_bus_set () {
  if (used_bus == 1) {
    CAN_cfg.speed = (CAN_speed_t)can_config->can1_speed;               // init the CAN bus (pins and baudrate)
    CAN_cfg.tx_pin_id = (gpio_num_t)can_config->can1_tx;
    CAN_cfg.rx_pin_id = (gpio_num_t)can_config->can1_rx;
  } else {
    CAN_cfg.speed = (CAN_speed_t)can_config->can2_speed;               // init the CAN bus (pins and baudrate)
    CAN_cfg.tx_pin_id = (gpio_num_t)can_config->can2_tx;
    CAN_cfg.rx_pin_id = (gpio_num_t)can_config->can2_rx;
  }
}

void can_init (CS_CONFIG_t *config) {
  can_config = config;
  can_bus_set ();
  // create a generic RTOS queue for CAN receiving, with 10 positions
  CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
  if (CAN_cfg.rx_queue == 0) {
    if (can_config->mode_debug) Serial.println("Can't create CANbus buffer. Stopping");
    while (1);
  }
  if (can_config->mode_debug) Serial.println("CAN starting ...");
  ESP32Can.CANInit();                              // initialize CAN Module
}

void can_send (CAN_frame_t *frame, uint8_t bus) {
  if (bus != used_bus) {
    used_bus = bus;
    ESP32Can.CANStop();
    can_bus_set ();
    ESP32Can.CANInit();                              // initialize CAN Module
  }
  ESP32Can.CANWriteFrame (frame);
}

void can_send_flow (uint16_t requestId) {
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
  can_send (&flow, used_bus);
}

boolean can_receive (CAN_frame_t *rx_frame) {
  return xQueueReceive (CAN_cfg.rx_queue, rx_frame, (TickType_t)0) == pdTRUE ? true : false;
}

// convert a CAN_frame to readable hex output format
String canFrameToString(CAN_frame_t &frame) {
  String dataString = String(frame.MsgID, HEX) + ",";
  for (int i = 0; i < frame.FIR.B.DLC; i++) {
    dataString += getHex(frame.data.u8[i]);
  }
  dataString += "\n";
  return dataString;
}
