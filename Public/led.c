#ifdef __cplusplus
extern "C" {
#endif
	#include "led.h"
	// Private variables
	uint8_t led_values[LEDRn];
	static uint8_t gamma_table[LEDPWM_CNT_TOP + 1];
	void ledpwm_init(void);
	void LED_Init(void)
	{	
		ledpwm_init();

	}
	void ledpwm_init(void) {
		memset((uint8_t*)led_values, 0, sizeof(led_values));
		for (uint16_t i = 0;i < (LEDPWM_CNT_TOP + 1);i++) {
//			gamma_table[i] = (int)roundf(powf((float)i / (float)LEDPWM_CNT_TOP, 1.0f / 0.45f) * (float)LEDPWM_CNT_TOP);
		}
	}
	/*
	 * Set the intensity for one led. The intensity value is mapped to a PWM value
	 * according to human luminance perception.
	 *
	 * Intensity range is 0.0f to 1.0f
	 */
	void ledpwm_set_intensity(uint8_t led, float intensity) {
		if (led >= LEDRn) {
			return;
		}

		if (intensity < 0.0f) {
			intensity = 0.0f;
		}

		if (intensity > 1.0f) {
			intensity = 1.0f;
		}

		led_values[led] = gamma_table[(int)(intensity * LEDPWM_CNT_TOP)];
	}

	void ledpwm_led_on(uint8_t led) {
		if (led >= LEDRn) {
			return;
		}

		led_values[led] = LEDPWM_CNT_TOP;
	}

	void ledpwm_led_off(uint8_t led) {
		if (led >= LEDRn) {
			return;
		}

		led_values[led] = 0;
	}
	/*
	 * Call this function as fast as possible, with a deterministic rate.
	 */
	void ledpwm_update_pwm(void) {
		static uint16_t cnt = 0;
		cnt++;
		if (cnt == LEDPWM_CNT_TOP) {
			cnt = 0;
		}
		if (cnt >= led_values[LED_STATUS]) {
			STARBOT_LED_STATUS_Off();
		} else {
			STARBOT_LED_STATUS_On();
		}
//		if (cnt >= led_values[LED_RUN]) {
//			STARBOT_LED_RUN_On();
//		} else {
//			STARBOT_LED_RUN_Off();
//		}
//		if (cnt >= led_values[LED_FAULT]) {
//			STARBOT_LED_FAULT_On();
//		} else {
//			STARBOT_LED_FAULT_Off();
//		}	
	}

#ifdef __cplusplus
}
#endif
