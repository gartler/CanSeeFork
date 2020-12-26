#include "leds.h"

static CS_CONFIG_t *leds_config;

void leds_init()
{
  leds_config = getConfig();
  if (leds_config->mode_leds == 0)
  {
    return;
  }
  else if (leds_config->mode_leds == LED_SINGLE)
  {
    pinMode(LED_BUILTIN, OUTPUT);
    led_set(LED_BUILTIN, true);
    delay(200);
    led_set(LED_BUILTIN, false);
  }
  else
  {
    // setup LED's
    pinMode(LED_WHITE, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    digitalWrite(LED_WHITE, LED_OFF);
    digitalWrite(LED_YELLOW, LED_OFF);
    digitalWrite(LED_RED, LED_OFF);
    digitalWrite(LED_GREEN, LED_OFF);
    digitalWrite(LED_BLUE, LED_OFF);

    // setup PWM to dim some of the LED's that may stay "always on"
    // like "power" and "bluetooth"
    ledcSetup(0, 5000, 8);
    ledcWrite(0, 250);

    // startup sequence and RED (=power) on
    led_set(LED_BLUE, true);
    delay(200);
    led_set(LED_BLUE, false);
    led_set(LED_GREEN, true);
    delay(200);
    led_set(LED_GREEN, false);
    led_set(LED_RED, true);
    delay(200);
    led_set(LED_RED, false);
    led_set(LED_YELLOW, true);
    delay(200);
    led_set(LED_YELLOW, false);
    led_set(LED_WHITE, true);
    delay(200);
    led_set(LED_WHITE, false);
    led_set(LED_RED, true);
  }
}

void led_set(unsigned led, bool on)
{
  static bool singleLedState = false;
  static bool blueState = false;
  if (leds_config->mode_leds == 0)
  {
    return;
  }
  else if (leds_config->mode_leds == LED_SINGLE)
  {
    // single LED compleely ignores absolute LED state, just switches
    // blue is a special case as it is continuously updated by connection state
    if (led == LED_BLUE)
    {
      // if blue only toggle on change
      if (on != blueState)
      {
        digitalWrite(LED_BUILTIN, (singleLedState = !singleLedState) ? LED_SINGLE_ON : LED_SINGLE_OFF);
        blueState = on;
      }
    }
    else
    {
      // else just toggle
      digitalWrite(LED_BUILTIN, (singleLedState = !singleLedState) ? LED_SINGLE_ON : LED_SINGLE_OFF);
    }
  }
  else
  {
    // full leds
    if ((led == LED_BLUE) || (led == LED_RED))
    {
      // dimmed for blue and red
      if (on)
      {
        ledcAttachPin(led, 0);
      }
      else
      {
        pinMatrixOutDetach(led, false, false);
        // this method is not yet in the actual build!
        //ledcDetachPin(LED_BLUE);
      }
    }
    else
    {
      // on of for others
      digitalWrite(led, on ? LED_ON : LED_OFF);
    }
  }
}

void toggleAliveLed()
{
  static bool aliveToggle = false;
  led_set(
      (leds_config->mode_leds == LED_SINGLE) ? LED_BUILTIN : LED_RED,
      (aliveToggle = !aliveToggle));
}
