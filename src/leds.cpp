/**
* @file leds.cpp
* @brief Abstraction for different LED configurations.
*/

#include "leds.h"

static CS_CONFIG_t *leds_config;

/**
 * Initialize LED subsystem
 */
void leds_init()
{
	leds_config = getConfig();
	if (leds_config->mode_leds == 0)
	{
		return;
	}
	else if (leds_config->mode_leds == LED_SINGLE)
	{
		pinMode(LED_SINGLE_PIN, OUTPUT);
		pinMode(LED_SINGLE_PIN2, OUTPUT);
		for (int i = 0; i < 3; i++)
		{
			led_set(LED_SINGLE_PIN, true);
			delay(200);
			led_set(LED_SINGLE_PIN, false);
			delay(200);
		}
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

/**
 * set LED on or off. On or off might not be literally implemented. For single LED
 * opearion, any call may for instance toggle the state.
 * @param led GPIO of the LED
 * @param on true for on. false for off
 */
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
		// single LED completely ignores absolute LED state, just switches
		// blue is a special case as it is continuously updated by connection state
		if (led == LED_BLUE)
		{
			// if blue only toggle on change
			if (on != blueState)
			{
				digitalWrite(LED_SINGLE_PIN, (singleLedState = !singleLedState) ? LED_SINGLE_ON : LED_SINGLE_OFF);
				digitalWrite(LED_SINGLE_PIN2, (singleLedState) ? LED_SINGLE_OFF : LED_SINGLE_ON);
				blueState = on;
			}
		}
		else
		{
			// else just toggle
			digitalWrite(LED_SINGLE_PIN, (singleLedState = !singleLedState) ? LED_SINGLE_ON : LED_SINGLE_OFF);
			digitalWrite(LED_SINGLE_PIN2, (singleLedState) ? LED_SINGLE_OFF : LED_SINGLE_ON);
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

/**
 * Special function to toggle the alive LED. This function will be called
 * every 1000 ms by the main loop.
 */
void toggleAliveLed()
{
	static bool aliveToggle = false;
	led_set(
		(leds_config->mode_leds == LED_SINGLE) ? LED_SINGLE_PIN : LED_RED,
		(aliveToggle = !aliveToggle));
}
