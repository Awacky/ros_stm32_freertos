#include "servo.h"
Hw_TIM_TypeDef  *SERVO_TIM[SERVOn] = {STARBOT_SERVO1_TIM, STARBOT_SERVO2_TIM,STARBOT_SERVO3_TIM,STARBOT_SERVO4_TIM};
const uint16_t  SERVOx_CHANNEL[SERVOn] = {STARBOT_SERVO1_CHANNEL, STARBOT_SERVO2_CHANNEL,STARBOT_SERVO3_CHANNEL, STARBOT_SERVO4_CHANNEL};
static int clockwise = 0;
static int counterclockwise = 0;
static int brake_state = 0;
static Servo privateServo1(SERVO1);
static Servo privateServo2(SERVO2);
static Servo privateServo3(SERVO3);
static Servo privateServo4(SERVO4);
Servo::Servo():setData(0.0),initDone(0)
{
	
}
Servo::Servo(Servo_TypeDef servo_):setData(0.0),initDone(0)
{
	servo = servo_ ; 
}
/// \ Steering gear name transfer function
/// \ return nothing
void Servo::update(Servo_TypeDef servo_)
{ 
	servo = servo_ ; 
}
/// \ Steering gear initialization function
/// \ return nothing
void Servo::init(void)
{	
	ServoInit(this->servo,0,0);
	initDone = 1;
}

/// \ Rotation Angle function
///	\ angle: rotation Angle(бу) 0-360
/// \ return nothing
uint16_t Servo::pos(uint16_t angle)
{ 
	float angle_s = 0.0;
	if(initDone){
		angle_s =(((angle+setData)*1000.0)/90.0) + 500;
		if(angle_s<500)
			angle_s = 500;
		if(angle_s>2500)
			angle_s = 2500;
		TIM_SetCompareX(SERVO_TIM[this->servo],SERVOx_CHANNEL[this->servo],angle_s);
	}
	return ((int)angle_s);
}
/// \ Movement function
///	\ pwm: PWM value
/// \ return nothing
void Servo::spinBrk(uint16_t esc_median,int pwm,int is_direction)
{ 
	if(initDone){
		if(is_direction == -1){
			pwm = -pwm;
		}
		if(pwm>0){
		  clockwise = 1;
		  brake_state = clockwise&counterclockwise;
		  counterclockwise = 0;
		}
		if(pwm<0){
		  counterclockwise = 1;
		  brake_state = clockwise&counterclockwise;
		  clockwise = 0;
		}
		escData = esc_median + pwm;
		if(brake_state){
			TIM_SetCompareX(SERVO_TIM[this->servo],SERVOx_CHANNEL[this->servo],esc_median);
			TIM_SetCompareX(SERVO_TIM[this->servo],SERVOx_CHANNEL[this->servo],escData);
		}else{
			TIM_SetCompareX(SERVO_TIM[this->servo],SERVOx_CHANNEL[this->servo],escData);
		}
	}
}
/// \ Movement function
///	\ pwm: PWM value
/// \ return nothing
void Servo::spin(uint16_t esc_median,int pwm,int is_direction)
{ 
	if(initDone){
		escData = abs(esc_median + pwm * is_direction);
		TIM_SetCompareX(SERVO_TIM[this->servo],SERVOx_CHANNEL[this->servo],escData);
	}
}
/// \ Update the redirection initialization function
///	\ servo_data: Turn to initial Settings Angle(бу)
/// \ return nothing
void Servo::updateConstants(uint8_t servo_data)
{
    setData = servo_data;
}

uint8_t Servo::getInitDone(void)
{
	return initDone;
}

#ifdef __cplusplus
extern "C" {
#endif
	
	
	
	
	
#ifdef __cplusplus
}
#endif





