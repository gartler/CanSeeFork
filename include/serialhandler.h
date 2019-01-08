#ifndef _SERIALHANDLER_H_
#define _SERIALHANDLER_H_

#include <Arduino.h>

#include "config.h"

void serial_init (CS_CONFIG *config, void (*p)());
void writeOutgoingSerial (String o);
void readIncomingSerial (String &readBuffer);

#endif
