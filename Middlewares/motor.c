#include "motor.h"
static Hw_GPIO_TypeDef* MOTOR_A_PORT[MOTORn] 				= {STARBOT_MOTOR1_A_GPIO_PORT, STARBOT_MOTOR2_A_GPIO_PORT, 
														STARBOT_MOTOR3_A_GPIO_PORT,STARBOT_MOTOR4_A_GPIO_PORT};
static Hw_GPIO_TypeDef* MOTOR_B_PORT[MOTORn] 				= {STARBOT_MOTOR1_B_GPIO_PORT, STARBOT_MOTOR2_B_GPIO_PORT, 
														STARBOT_MOTOR3_B_GPIO_PORT,STARBOT_MOTOR4_B_GPIO_PORT};
static Hw_TIM_TypeDef* MOTOR_TIM[MOTORn] 					= {STARBOT_MOTOR1_TIM, STARBOT_MOTOR2_TIM, \
														STARBOT_MOTOR3_TIM, STARBOT_MOTOR4_TIM};
static const Hw_TIM_CHA_TypeDef  MOTOR_A_CHANNEL[MOTORn] 	= {STARBOT_MOTOR1_A_CHANNEL, STARBOT_MOTOR2_A_CHANNEL, \
														STARBOT_MOTOR3_A_CHANNEL, STARBOT_MOTOR4_A_CHANNEL};
static const Hw_TIM_CHA_TypeDef  MOTOR_B_CHANNEL[MOTORn] 	= {STARBOT_MOTOR1_B_CHANNEL, STARBOT_MOTOR2_B_CHANNEL, \
														STARBOT_MOTOR3_B_CHANNEL, STARBOT_MOTOR4_B_CHANNEL};
static const uint16_t  MOTOR_A_PIN[MOTORn] 				= {STARBOT_MOTOR1_A_PIN, STARBOT_MOTOR2_A_PIN, \
														STARBOT_MOTOR3_A_PIN, STARBOT_MOTOR4_A_PIN,};
static const uint16_t  MOTOR_B_PIN[MOTORn] 				= {STARBOT_MOTOR1_B_PIN, STARBOT_MOTOR2_B_PIN,\
														STARBOT_MOTOR3_B_PIN, STARBOT_MOTOR4_B_PIN};

void MotorSpin(const vMotorStr* Src){
	int msDir_t = 0;
	msDir_t = Src->dutyVal * Src->mDir;
	switch(Src->mDirveType){
		case M_A4950:{
			if(msDir_t>0){
				TIM_SetCompareX(MOTOR_TIM[Src->mName],  MOTOR_A_CHANNEL[Src->mName],0);
				TIM_SetCompareX(MOTOR_TIM[Src->mName],  MOTOR_B_CHANNEL[Src->mName],abs(Src->dutyVal));
			}else if(msDir_t<0){
				TIM_SetCompareX(MOTOR_TIM[Src->mName],  MOTOR_A_CHANNEL[Src->mName],abs(Src->dutyVal));
				TIM_SetCompareX(MOTOR_TIM[Src->mName],  MOTOR_B_CHANNEL[Src->mName],0);
			}else{
				TIM_SetCompareX(MOTOR_TIM[Src->mName],  MOTOR_A_CHANNEL[Src->mName],0);
				TIM_SetCompareX(MOTOR_TIM[Src->mName],  MOTOR_B_CHANNEL[Src->mName],0);
			}	
		}break;
		case M_BTN797X:{
			if(msDir_t>0){
				MOTOR_B_PORT[Src->mName]->Hw_GPIO_Set 	= MOTOR_B_PIN[Src->mName];
				TIM_SetCompareX(MOTOR_TIM[Src->mName],  MOTOR_A_CHANNEL[Src->mName],abs(Src->dutyVal));
			}else if(msDir_t<0){
				MOTOR_B_PORT[Src->mName]->Hw_GPIO_ReSet = MOTOR_B_PIN[Src->mName];
				TIM_SetCompareX(MOTOR_TIM[Src->mName],  MOTOR_A_CHANNEL[Src->mName],abs(Src->dutyVal));
			}else{
				MOTOR_B_PORT[Src->mName]->Hw_GPIO_ReSet = MOTOR_B_PIN[Src->mName];
				TIM_SetCompareX(MOTOR_TIM[Src->mName],  MOTOR_A_CHANNEL[Src->mName],0);
			}
		}break;
		case M_DCM_4WD:{
			TIM_MotorSpin(Src->mName,Src->dutyVal,abs(Src->dutyVal));
		}break;
	}
}
void canMotorSpinSend(const vMotorStr* Src){
	
}