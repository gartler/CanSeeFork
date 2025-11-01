#ifndef _UTILS_H_
#define _UTILS_H_

#include <Arduino.h>

String getHexSimple(uint8_t num);
String getHex(uint32_t num);
uint32_t hexToDec(String hexString);

#endif
