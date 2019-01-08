#ifndef _LEDS_H_
#define _LEDS_H_

#include <Arduino.h>

#include "config.h"

#define LED_WHITE 26    // ISO-TP frame incoming
#define LED_YELLOW 27   // free fame incoming
#define LED_RED 32      // power
#define LED_GREEN 33    // serial incoming request
#define LED_BLUE 25     // bluetooth connected
#define LED_ON LOW      // active LOW
#define LED_OFF HIGH

void leds_init (CS_CONFIG *config);
void led_set (unsigned led, bool on);

#endif
