#include "config.h"

CONFIG config;

void setConfigDefault () {
  config.magicnumber = 0x0caacee0;                // does that read CanSee?
  config.version = 1;                             // change if length of config changes
  config.mode_serial = 1;
  config.mode_bluetooth = 1;
  config.mode_wifi = WIFI_SOFTAP;
  config.mode_debug = 0xff;
  config.mode_leds = 0;
  strcpy (config.name_bluetooth, "CANSee");
  strcpy (config.pin_bluetooth, "1234");          // not implemented in framework yet
  strcpy (config.ssid_ap, "CANSee");
  strcpy (config.password_ap, "CANSeeMe");
  strcpy (config.ssid_station, "Home");
  strcpy (config.password_station, "Password");
}

CONFIG *getConfigFromEeprom () {
  if (!EEPROM.begin (sizeof (CONFIG)))
  {
    Serial.println ("failed to initialise EEPROM for reading");
    setConfigDefault ();
    return &config;
  }
  if (EEPROM.readBytes (0, &config, sizeof (CONFIG)) != sizeof (CONFIG) || config.magicnumber != 0x0caacee0) {
    Serial.println ("Not a valid EEPROM record");
    setConfigToEeprom (true);
  }
  EEPROM.end ();
  return &config;
}

void setConfigToEeprom (bool reset) {
  if (!EEPROM.begin (sizeof (CONFIG)))
  {
    Serial.println ("failed to initialise EEPROM for writing");
    return;
  }
  if (reset) setConfigDefault ();
  EEPROM.writeBytes (0, &config, sizeof (CONFIG));
  EEPROM.commit ();
  EEPROM.end ();
  return;
}
