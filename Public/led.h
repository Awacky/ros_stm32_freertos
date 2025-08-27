#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _LED_H_
	#define _LED_H_
	
	#include "hw_config.h"

	void LED_Init(void);
	void ledpwm_init(void);
	void ledpwm_set_intensity(uint8_t led, float intensity);
	void ledpwm_led_on(uint8_t led);
	void ledpwm_led_off(uint8_t led);
	void ledpwm_update_pwm(void);
	
	
	#endif //_LED_H_
#ifdef __cplusplus
}
#endif


