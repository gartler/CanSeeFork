#include "leds.h"

static CS_CONFIG_t *leds_config;

void leds_init () {
  leds_config = getConfig ();
  if (!leds_config->mode_leds) return;

  // setup LED's
  pinMode(LED_WHITE , OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED   , OUTPUT);
  pinMode(LED_GREEN , OUTPUT);
  pinMode(LED_BLUE  , OUTPUT);

  digitalWrite(LED_WHITE,  LED_OFF);
  digitalWrite(LED_YELLOW, LED_OFF);
  digitalWrite(LED_RED,    LED_OFF);
  digitalWrite(LED_GREEN,  LED_OFF);
  digitalWrite(LED_BLUE,   LED_OFF);

  // setup PWM to dim some of the LED's that may stay "always on"
  // like "power" and "bluetooth"
  ledcSetup(0, 5000, 8);
  ledcWrite(0, 250);

  // startup sequence and RED (=power) on
  led_set(LED_BLUE  , true); delay(200); led_set(LED_BLUE  , false);
  led_set(LED_GREEN , true); delay(200); led_set(LED_GREEN , false);
  led_set(LED_RED   , true); delay(200); led_set(LED_RED   , false);
  led_set(LED_YELLOW, true); delay(200); led_set(LED_YELLOW, false);
  led_set(LED_WHITE , true); delay(200); led_set(LED_WHITE , false);
  led_set(LED_RED, true);
}

void led_set (unsigned led, bool on) {
  if (!leds_config->mode_leds) return;
  if ((led == LED_BLUE) || (led == LED_RED)) {
    if (on) {
      ledcAttachPin(led, 0);
    } else {
      pinMatrixOutDetach(led, false, false);
      // this method is not yet in the actual build!
      //ledcDetachPin(LED_BLUE);
    }
  } else {
    digitalWrite(led, on ? LED_ON : LED_OFF);
  }
}
