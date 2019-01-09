#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <EEPROM.h>

#define WIFI_STATION 1
#define WIFI_SOFTAP  2

#define DEBUG_COMMAND_FF      0x08
#define DEBUG_COMMAND         0x04
#define DEBUG_BUS_RECEIVE_FF  0x01
#define DEBUG_COMMAND_ISO     0x10

// Config ********************************************************************
// structure that defines the firmware's configuration
typedef struct {
  uint32_t magicnumber;                    // does that read CanSee?
  byte version;
  byte mode_serial;
  byte mode_bluetooth;
  byte mode_wifi;
  byte mode_debug;
  byte mode_leds;
  char name_bluetooth [32];
  char pin_bluetooth [8];
  char ssid_ap [32];
  char password_ap [16];
  char ssid_station [32];
  char password_station [16];
  byte can1_rx;
  byte can1_tx;
  byte can1_speed;
  byte can2_rx;
  byte can2_tx;
  byte can2_speed;
} CS_CONFIG_t;

CS_CONFIG_t *getConfigFromEeprom ();
void setConfigToEeprom (bool reset);

#endif
