#ifndef _SERVO_H_
#define _SERVO_H_

#include "hw_config.h"

class Servo 
{
	public:
		Servo();
		Servo(Servo_TypeDef servo_);
		void update(Servo_TypeDef servo_);
		void init(void);
		uint8_t getInitDone(void);
		uint16_t pos(uint16_t angle);
		void spin(uint16_t esc_median,int pwm,int is_direction);
		void spinBrk(uint16_t esc_median,int pwm,int is_direction);
		void updateConstants(uint8_t servo_data);
	private:
		uint8_t initDone;
		uint16_t escData;
		Servo_TypeDef servo;
		uint8_t setData;
};

#endif //_SERVO_H_
