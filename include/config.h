#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <EEPROM.h>

#define WIFI_STATION 1
#define WIFI_SOFTAP 2

#define DEBUG_COMMAND_FF 0x08     //     unused
#define DEBUG_COMMAND 0x04 // (*) debug all command activity
#define DEBUG_BUS_RECEIVE_ISO 0x02 
#define DEBUG_BUS_RECEIVE_FF 0x01 //     debug all incoming freeframes
#define DEBUG_COMMAND_ISO 0x10	  // (*) debug all incoming and outgoing ISOTP-frames
#define DEBUG_FLUSH_SERIAL 0x80	  // experimental. Avoid possible crash because of serial overwhelming

#define LED_SINGLE 1
#define LED_MULTI 2 // actually, anything but 0 and 1, but 2 is defined

/**
 * @brief Structure that defines the firmware's configuration
 */
typedef struct
{
	uint32_t magicnumber;				/**< @brief Magic number indicating the EEPROM blob is valid */
	byte version;						/**< @brief Version of the EEPROM blob */
	byte mode_serial;					/**< @brief Serial mode. o=off, *=on */
	byte mode_bluetooth;				/**< @brief Bluetooth mode. 0=off, 1=on, 2=connected */
	byte mode_wifi;						/**< @brief Wifi mode. 0=off, 1=station, 2=access point */
	byte mode_debug;					/**< @brief Debug settings, see defines */
	byte mode_leds;						/**< @brief LED setting. 0=off, 1=SINGLE, *=MULTI */
	char name_bluetooth[32];			/**< @brief Bluetooth name */
	char pin_bluetooth[8];				/**< @brief PIN for Bluetooth pairing (unused) */
	char ssid_ap[32];					/**< @brief WiFi SSID if in access point mode*/
	char password_ap[16];				/**< @brief WiFi password for access point mode*/
	char ssid_station[32];				/**< @brief WiFi SSID if in station mode */
	char password_station[16];			/**< @brief WiFi password for station mode */
	byte can0_rx;						/**< @brief GPIO of RX of CANbus 0 */
	byte can0_tx;						/**< @brief GPIO of TX of CANbus 0 */
	uint16_t can0_speed;				/**< @brief speed of CANbus 0*/
	byte can1_rx;						/**< @brief GPIO of RX of CANbus 1 */
	byte can1_tx;						/**< @brief GPIO of TX of CANbus 1 */
	uint16_t can1_speed;				/**< @brief speed of CANbus 1 (unused) */
	//** past this line will NOT be initialized by EEPROM but it global data
	uint8_t bus;					  /**< @brief bus (unused, only 0 allowed */
	void (*command_handler)();		  /**< @brief command handler */
	void (*output_handler)(String o); /**< @brief output handler */
	uint16_t boot_count;			  /**< @brief Times the device has been booted */
} CS_CONFIG_t;

CS_CONFIG_t *getConfig();
void setConfigToEeprom(bool reset);

#endif
