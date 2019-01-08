#include "config.h"

CS_CONFIG cs_config;

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
}

CS_CONFIG *getConfigFromEeprom () {
  if (!EEPROM.begin (sizeof (CS_CONFIG)))
  {
    Serial.println ("failed to initialise EEPROM for reading");
    setConfigDefault ();
    return &cs_config;
  }
  if (EEPROM.readBytes (0, &cs_config, sizeof (CS_CONFIG)) != sizeof (CS_CONFIG) || cs_config.magicnumber != 0x0caacee0) {
    Serial.println ("Not a valid EEPROM record");
    setConfigToEeprom (true);
  }
  EEPROM.end ();
  return &cs_config;
}

void setConfigToEeprom (bool reset) {
  if (!EEPROM.begin (sizeof (CS_CONFIG)))
  {
    Serial.println ("failed to initialise EEPROM for writing");
    return;
  }
  if (reset) setConfigDefault ();
  EEPROM.writeBytes (0, &cs_config, sizeof (CS_CONFIG));
  EEPROM.commit ();
  EEPROM.end ();
  return;
}
