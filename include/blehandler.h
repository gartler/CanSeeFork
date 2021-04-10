#ifndef _BLEHANDLER_H_
#define _BLEHANDLER_H_

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "config.h"
#include "leds.h"

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Repository included libraries includes, see ./lib/ ************************

void ble_init ();
void writeOutgoingBle (String o);
void readIncomingBle (String &readBuffer);
void checkStateBle ();

#endif
