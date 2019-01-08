#ifndef _WIFIHANDLER_H_
#define _WIFIHANDLER_H_

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"

#define MAX_SRV_CLIENTS 1

void wifi_init (CS_CONFIG *config, void (*p)());
void writeOutgoingWiFi (String o);
void readIncomingWiFi (String &readBuffer);

#endif
