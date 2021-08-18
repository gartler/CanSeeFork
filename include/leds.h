#ifndef _LEDS_H_
#define _LEDS_H_

#include <Arduino.h>

#include "config.h"

#define LED_BLUE 25   // BT connected, connected to WiFi, client has connected
#define LED_GREEN 33  // CAN traffic, either sending or receiving
#define LED_RED 32    // power indicator
#define LED_YELLOW 27 // reserved for future use (maybe: off = BT mode, on = WiFi Client, blink = WiFi AP)
#define LED_WHITE 26  // reserved for future use

#define LED_SINGLE_PIN 2
#define LED_SINGLE_PIN2 17

#define LED_ON HIGH // Active HIGH
#define LED_OFF LOW

#define LED_SINGLE_ON HIGH
#define LED_SINGLE_OFF LOW

void leds_init();
void led_set(unsigned led, bool on);
void toggleAliveLed();

#endif
