/**
* @file config.cpp
* @brief This module handles the EEPROM settings.
*/

#include "config.h"
#include "canhandler.h"

static CS_CONFIG_t cs_config;
static bool fetched = false;

/**
 * default for version 3
 */
static void setConfigDefault_3()
{
	cs_config.version = 3; // change if length of config changes
	cs_config.can0_rx = GPIO_NUM_4;
	cs_config.can0_tx = GPIO_NUM_5;
	cs_config.can0_speed = (uint16_t)CAN_SPEED_500KBPS; // (uint16_t)CAN_SPEED_500KBPS;
	cs_config.can1_rx = GPIO_NUM_18;
	cs_config.can1_tx = GPIO_NUM_19;
	cs_config.can1_speed = (uint16_t)CAN_SPEED_250KBPS;
}

/**
 * Default for version 4
 */
static void setConfigDefault_4(bool reset_boot_count)
{
	cs_config.version = 4; // change if length of config changes
	if (reset_boot_count)
		cs_config.boot_count = 0;
}

/**
 * setConfigRam NULLS the pointers to code. THese should never be fetched from EEPROM
 * It also sets the fetched status to true.
 */
static void setConfigRam()
{
	cs_config.bus = 0;
	cs_config.command_handler = NULL;
	cs_config.output_handler = NULL;
	fetched = true;
}

/**
 * Reset the config object to default values
 */
static void setConfigDefault()
{
	cs_config.magicnumber = 0x0caacee0;				// does that read CanSee?
	cs_config.version = 1;							// change if length of config changes
	cs_config.mode_serial = 1;						//
	cs_config.mode_bluetooth = 0;					//
	cs_config.mode_wifi = 2;						// WIFI_SOFTAP or WIFI_STATION
	cs_config.mode_debug = 0x04;					// flush serial, debug ISO command and bus, debug command
	cs_config.mode_leds = 0;						// single LED
	strcpy(cs_config.name_bluetooth, "CANSee");		// name
	strcpy(cs_config.pin_bluetooth, "1234");		// not implemented in framework yet
	strcpy(cs_config.ssid_ap, "CANSee");			//
	strcpy(cs_config.password_ap, "CANSeeMe");		//
	strcpy(cs_config.ssid_station, "Home");			//
	strcpy(cs_config.password_station, "Username"); //
	setConfigDefault_3();							// additional fields for version 3
	setConfigDefault_4(false);						// additional fields for version 4
	setConfigRam();
}

CS_CONFIG_t *getConfig()
{
	// if already fetched, return the object from RAM
	if (fetched)
		return &cs_config;

	// if not, open the EEPROM
	if (!EEPROM.begin(sizeof(CS_CONFIG_t)))
	{
		Serial.println("failed to initialise EEPROM for reading");
		setConfigDefault(); // create a default config
		setConfigRam();		// cleanup the RAM and set fetched
		return &cs_config;	// return a pointer to the config object
	}

	// read the EEPROM and check the magic number
	if (EEPROM.readBytes(0, &cs_config, sizeof(CS_CONFIG_t)) != sizeof(CS_CONFIG_t) || cs_config.magicnumber != 0x0caacee0)
	{
		Serial.println("Not a valid EEPROM record");
		setConfigToEeprom(true); // reset the RAM and write it to EEPROM
	}

	// so here we have a valid cs_config. If we read an old structure, update
	if (cs_config.version < 3)
	{
		Serial.println("EEPROM structure changed");
		EEPROM.end();
		setConfigDefault_3();
		setConfigToEeprom(false);
	}
	else if (cs_config.version < 4)
	{
		Serial.println("EEPROM structure changed");
		EEPROM.end();
		setConfigDefault_4(true);
		setConfigToEeprom(false);
	}
	else
	{
		EEPROM.end();
	}

	// cleanup the RAM and set fetched
	setConfigRam();
	return &cs_config;
}

/**
 * Write the RAM config opbecjt to EEPROM, optionally resetting the config
 * to it's default settings
 * @param reset If true, reset the config to it's default settings before writing
 */
void setConfigToEeprom(bool reset)
{
	if (!EEPROM.begin(sizeof(CS_CONFIG_t)))
	{
		Serial.println("failed to initialise EEPROM for writing");
		return;
	}
	if (reset)
		setConfigDefault();
	EEPROM.writeBytes(0, &cs_config, sizeof(CS_CONFIG_t));
	EEPROM.commit();
	EEPROM.end();
	return;
}
