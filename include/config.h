#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <EEPROM.h>

#define WIFI_STATION 1
#define WIFI_SOFTAP  2


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
} CS_CONFIG;

CS_CONFIG *getConfigFromEeprom ();
void setConfigToEeprom (bool reset);

#endif
