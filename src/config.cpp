#include "config.h"
#include "CAN_config.h"

static CS_CONFIG_t cs_config;

void setConfigDefault_2 () {
  cs_config.version = 2;                             // change if length of config changes
  cs_config.can1_rx = GPIO_NUM_4;
  cs_config.can1_tx = GPIO_NUM_5;
  cs_config.can1_speed = (byte)CAN_SPEED_500KBPS;
  cs_config.can2_rx = GPIO_NUM_6;                    // reserved for future use
  cs_config.can2_tx = GPIO_NUM_7;
  cs_config.can2_speed = (byte)CAN_SPEED_250KBPS;
}

void setConfigDefault () {
  cs_config.magicnumber = 0x0caacee0;                // does that read CanSee?
  cs_config.version = 1;                             // change if length of config changes
  cs_config.mode_serial = 1;
  cs_config.mode_bluetooth = 1;
  cs_config.mode_wifi = WIFI_SOFTAP;
  cs_config.mode_debug = 0xff;
  cs_config.mode_leds = 0;
  strcpy (cs_config.name_bluetooth, "CANSee");
  strcpy (cs_config.pin_bluetooth, "1234");          // not implemented in framework yet
  strcpy (cs_config.ssid_ap, "CANSee");
  strcpy (cs_config.password_ap, "CANSeeMe");
  strcpy (cs_config.ssid_station, "Home");
  strcpy (cs_config.password_station, "Password");
  setConfigDefault_2 ();
}

CS_CONFIG_t *getConfigFromEeprom () {
  if (!EEPROM.begin (sizeof (CS_CONFIG_t)))
  {
    Serial.println ("failed to initialise EEPROM for reading");
    setConfigDefault ();
    return &cs_config;
  }
  if (EEPROM.readBytes (0, &cs_config, sizeof (CS_CONFIG_t)) != sizeof (CS_CONFIG_t) || cs_config.magicnumber != 0x0caacee0) {
    Serial.println ("Not a valid EEPROM record");
    setConfigToEeprom (true);
  }
  if (cs_config.version == 1) {
    setConfigDefault_2 ();
  }
  EEPROM.end ();
  return &cs_config;
}

void setConfigToEeprom (bool reset) {
  if (!EEPROM.begin (sizeof (CS_CONFIG_t)))
  {
    Serial.println ("failed to initialise EEPROM for writing");
    return;
  }
  if (reset) setConfigDefault ();
  EEPROM.writeBytes (0, &cs_config, sizeof (CS_CONFIG_t));
  EEPROM.commit ();
  EEPROM.end ();
  return;
}
