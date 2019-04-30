#ifndef _LEDS_H_
#define _LEDS_H_

#include <Arduino.h>

#include "config.h"

#define LED_BLUE   25   // BT connected, connected to WiFi, client has connected
#define LED_GREEN  33   // CAN traffic, either sending or receiving
#define LED_RED    32   // power indicator
#define LED_YELLOW 27   // reserved for future use (maybe: off = BT mode, on = WiFi Client, blink = WiFi AP)
#define LED_WHITE  26   // reserved for future use
#define LED_ON LOW      // old:active LOW
#define LED_OFF HIGH

#define LED_SINGLE_ON HIGH
#define LED_SINGLE_OFF LOW

void leds_init ();
void led_set (unsigned led, bool on);

#endif
