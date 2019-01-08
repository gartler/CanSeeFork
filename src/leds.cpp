#include "leds.h"

CS_CONFIG *leds_config;

void leds_init (CS_CONFIG *config) {
  leds_config = config;
  if (!leds_config->mode_leds) return;

  // setup LED's
  pinMode (LED_WHITE , OUTPUT);
  pinMode (LED_YELLOW, OUTPUT);
  pinMode (LED_RED   , OUTPUT);
  pinMode (LED_GREEN , OUTPUT);
  pinMode (LED_BLUE  , OUTPUT);

  led_set (LED_WHITE , false);
  led_set (LED_YELLOW, false);
  led_set (LED_RED   , false);
  led_set (LED_GREEN , false);
  led_set (LED_BLUE  , false);

  // setup PWM to dim some of the LED's that may stay "always on"
  // like "power" and "bluetooth"
  ledcSetup(0, 5000, 8);
  ledcWrite(0, 250);

  // startup sequence and RED (=power) on
  led_set (LED_BLUE  , true); delay(200); led_set (LED_BLUE  , false);
  led_set (LED_GREEN , true); delay(200); led_set (LED_GREEN , false);
  led_set (LED_RED   , true); delay(200); led_set (LED_RED   , false);
  led_set (LED_YELLOW, true); delay(200); led_set (LED_YELLOW, false);
  led_set (LED_WHITE , true); delay(200); led_set (LED_WHITE , false);
  led_set (LED_RED, true);
}

void led_set (unsigned led, bool on) {
  if (!leds_config->mode_leds) return;
  if (led == LED_BLUE || LED_RED) {
    if (on) {
      ledcAttachPin(LED_BLUE, 0);
    } else {
      pinMatrixOutDetach(LED_BLUE, false, false);
      // this method is not yet in the actual build!
      //ledcDetachPin(LED_BLUE);
    }
  } else {
    digitalWrite(led, on ? LED_ON : LED_OFF);
  }
}
