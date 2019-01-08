#include "leds.h"

CS_CONFIG *leds_config;

void leds_init (CS_CONFIG *config) {
  leds_config = config;

  if (leds_config->mode_leds) {
    // setup LED's
    pinMode(LED_WHITE, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    // they are active low ...
    digitalWrite(LED_WHITE, LED_OFF);
    digitalWrite(LED_YELLOW, LED_OFF);
    digitalWrite(LED_RED, LED_OFF);
    digitalWrite(LED_GREEN, LED_OFF);
    digitalWrite(LED_BLUE, LED_OFF);

    // startup sequence
    digitalWrite(LED_BLUE, LED_ON);
    delay(200);
    digitalWrite(LED_BLUE, LED_OFF);
    digitalWrite(LED_GREEN, LED_ON);
    delay(200);
    digitalWrite(LED_GREEN, LED_OFF);
    digitalWrite(LED_RED, LED_ON);
    delay(200);
    digitalWrite(LED_RED, LED_OFF);
    digitalWrite(LED_YELLOW, LED_ON);
    delay(200);
    digitalWrite(LED_YELLOW, LED_OFF);
    digitalWrite(LED_WHITE, LED_ON);
    delay(200);
    digitalWrite(LED_WHITE, LED_OFF);

    // setup PWM to dim some of the LED's that may stay "always on"
    // like "power" and "bluetooth"
    ledcSetup(0, 5000, 8);
    ledcWrite(0, 250);

    // turn RED (=power) immediately on
    ledcAttachPin(LED_RED, 0);
  }
}

void led_set (unsigned led, bool on) {
  if (!leds_config->mode_leds) return;

  if (led == LED_BLUE) {
    if (on) {
      ledcAttachPin(LED_BLUE, 0);
    } else {
      pinMatrixOutDetach(LED_BLUE, false, false);
      // this method is not yet in the actual build!
      //ledcDetachPin(LED_BLUE);
    }
  } else {
    digitalWrite(led, on);
  }
}
