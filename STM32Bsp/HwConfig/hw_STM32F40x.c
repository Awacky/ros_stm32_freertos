#include "hw_config.h"
#ifdef __cplusplus
extern "C" {
#endif 
#ifndef BOOTLOADER
	const char Hardware_Ver_Static[] __attribute__((at(APP_ADDR+0x400))) = "STF407VG-V1.2.7";
	const char Software_Ver_Static[] __attribute__((at(APP_ADDR+0x410))) = SOFTWARE_VER;
	const uint32_t CheckFile_Static __attribute__((at(APP_CHECH))) = 0xFFFFFFFE;				//APP_ADDR+0x420
#endif
static void MC_PWMInit(uint8_t mName_t,uint32_t mPwm_Arr,uint32_t mPwm_Psc);
void SoftReset_fun(void){
	__set_FAULTMASK(1);//关闭所有中断
	NVIC_SystemReset();//复位函数
}
void hw_GPIO_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	//蜂鸣器
	RCC_AHB1PeriphClockCmd(STARBOT_BEEP_CLK, ENABLE);		//使能GPIOF时钟
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	
	GPIO_InitStructure.GPIO_Pin = STARBOT_BEEP_PIN;	
	GPIO_Init(STARBOT_BEEP_PORT, &GPIO_InitStructure);		//初始化GPIO
	STARBOT_BEEP_Off();
	
	//LED
	RCC_AHB1PeriphClockCmd(STARBOT_LED_STATUS_CLK|STARBOT_LED_RUN_CLK|STARBOT_LED_FAULT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = STARBOT_LED_STATUS_PIN;
	GPIO_Init(STARBOT_LED_STATUS_PORT, &GPIO_InitStructure); 
	STARBOT_LED_STATUS_Off();

	GPIO_InitStructure.GPIO_Pin = STARBOT_LED_RUN_PIN;
	GPIO_Init(STARBOT_LED_RUN_PORT, &GPIO_InitStructure); 
	STARBOT_LED_RUN_Off();

	GPIO_InitStructure.GPIO_Pin = STARBOT_LED_FAULT_PIN;
	GPIO_Init(STARBOT_LED_FAULT_PORT, &GPIO_InitStructure); 
	STARBOT_LED_FAULT_Off();
	#ifdef BOOTLOADER
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	#else
	//可控电源输出
	RCC_AHB1PeriphClockCmd(STARBOT_POWER_KEY1_CLK|STARBOT_POWER_KEY2_CLK, ENABLE);											//使能GPIOF时钟	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;																			//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;																			//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;																		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;																			//上拉
	
	GPIO_InitStructure.GPIO_Pin = STARBOT_POWER_KEY1_PIN;
	GPIO_Init(STARBOT_POWER_KEY1_PORT, &GPIO_InitStructure);																//初始化GPIO
	STARBOT_POWER_KEY1_Off();
	
	GPIO_InitStructure.GPIO_Pin = STARBOT_POWER_KEY2_PIN;
	GPIO_Init(STARBOT_POWER_KEY2_PORT, &GPIO_InitStructure);																//初始化GPIO
	STARBOT_POWER_KEY2_Off();
	
	//急停按钮
	RCC_AHB1PeriphClockCmd(STARBOT_EMERGENCY_STOP_CLK, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = STARBOT_EMERGENCY_STOP_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(STARBOT_EMERGENCY_STOP_PORT, &GPIO_InitStructure);

	//PS2
	RCC_AHB1PeriphClockCmd(STARBOT_PSTWO_CS_CLK|STARBOT_PSTWO_SCL_CLK|\
						   STARBOT_PSTWO_SCL_CLK|STARBOT_PSTWO_DO_CLK|\
						   STARBOT_PSTWO_CHECK_CLK, ENABLE);																//使能GPIOB时钟
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;																			//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;																			//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;																		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;																			//上拉

	GPIO_InitStructure.GPIO_Pin =  STARBOT_PSTWO_CS_PIN;		
	GPIO_Init(STARBOT_PSTWO_CS_PORT, &GPIO_InitStructure);		
	GPIO_SetBits(STARBOT_PSTWO_CS_PORT,STARBOT_PSTWO_CS_PIN);	

	GPIO_InitStructure.GPIO_Pin =  STARBOT_PSTWO_SCL_PIN;
	GPIO_Init(STARBOT_PSTWO_SCL_PORT, &GPIO_InitStructure);
	GPIO_SetBits(STARBOT_PSTWO_SCL_PORT,STARBOT_PSTWO_SCL_PIN);

	GPIO_InitStructure.GPIO_Pin =  STARBOT_PSTWO_DO_PIN;
	GPIO_Init(STARBOT_PSTWO_DO_PORT, &GPIO_InitStructure);
	GPIO_SetBits(STARBOT_PSTWO_DO_PORT,STARBOT_PSTWO_DO_PIN);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;																			//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;																		//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;																			//上拉
	
	GPIO_InitStructure.GPIO_Pin = STARBOT_PSTWO_DI_PIN; 
	GPIO_Init(STARBOT_PSTWO_DI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = STARBOT_PSTWO_CHECK_PIN;
	GPIO_Init(STARBOT_PSTWO_CHECK_PORT, &GPIO_InitStructure);
	
	//SoftwareIIC
	RCC_AHB1PeriphClockCmd(STARBOT_IMU_N_SCL_CLK|STARBOT_IMU_N_SDA_CLK|\
						   STARBOT_IMU_W_SCL_CLK|STARBOT_IMU_W_SDA_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_Pin = STARBOT_IMU_N_SCL_PIN;	
	GPIO_Init(STARBOT_IMU_N_SCL_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = STARBOT_IMU_W_SCL_PIN;	
	GPIO_Init(STARBOT_IMU_W_SCL_PORT, &GPIO_InitStructure);
	#endif
}
#ifndef BOOTLOADER
void watchdogInit(uint16_t xms){
	if(0==xms){
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
		IWDG_Enable();
	}else{
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		IWDG_SetPrescaler(IWDG_Prescaler_32);
		/* 47000/32Hz => 1.47  1ms*/
		IWDG_SetReload((uint16_t)(1.47*xms));
		watchdogReset();
		IWDG_Enable();
	}
}
void BaseBoard_TIM6_Init(void){	//10ms
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//84MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);  
	
	TIM_TimeBaseInitStructure.TIM_Period = 83; 	
	TIM_TimeBaseInitStructure.TIM_Prescaler=9999;  
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); 
	TIM_Cmd(TIM6,ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM_SetCompareX(void *tmr_x, uint16_t tmr_channel,uint32_t tmr_channel_value){
	TIM_TypeDef *p_t = (TIM_TypeDef *)tmr_x;
	switch(tmr_channel){
		case TIM_Channel_1:
			TIM_SetCompare1(p_t, tmr_channel_value);
			break;
		case TIM_Channel_2:
			TIM_SetCompare2(p_t, tmr_channel_value);
			break;
		case TIM_Channel_3:
			TIM_SetCompare3(p_t, tmr_channel_value);
			break;
		case TIM_Channel_4:
			TIM_SetCompare4(p_t, tmr_channel_value);
			break;
	}
}
void TIM_MotorSpin(uint8_t mName_t,int Dir_t,uint32_t Duty_t){
	switch(mName_t){
		case MOTOR1:{
			if(Duty_t==0){
				TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);		//强制设置为高电平
				TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);					//PE9  	CH1 L
				TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);					//PE8  	CH1 H   	GND -	
				
				TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);					//PE11 	CH2 L
				TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);					//PE10 	CH2 H    	GND -	
			}else if(Dir_t>0){
				TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);					//PE9  	CH1 L
				TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);					//PE8  	CH1 H 		GND -
				
				TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);	
				TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);					//PE11 	CH2 PWM  	VCC +
				TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);					//PE10 	CH2 L
				TIM_SetCompare2(TIM1, Duty_t);				
			} else {
				TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
				TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);					//PE9  	CH1 PWM  	VCC +
				TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);					//PE8  	CH1 L   
				TIM_SetCompare1(TIM1, Duty_t);
				
				TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);					//PE11 	CH2 L
				TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);					//PE10 	CH2 H    	GND -	
			}
		}break;
		case MOTOR2:{
			if(Duty_t==0){
				TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);					//PE13	CH3 L
				TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);					//PE12	CH3 H 		GND -
				
				TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Disable);					//PC6	CH1 L
				TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);					//PA7	CH1 H 		GND -
			}else if(Dir_t>0){
				TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);					//PE13	CH3 L
				TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);					//PE12	CH3 H 		GND -
				
				TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
				TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);					//PC6	CH1 PWM  	VCC +
				TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Disable);					//PA7	CH1 L
				TIM_SetCompare1(TIM8, Duty_t);
			} else {
				TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);				
				TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);					//PE13	CH3 PWM  	VCC +
				TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);					//PE12	CH3 L
				TIM_SetCompare3(TIM1, Duty_t);
				
				TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Disable);					//PC6	CH1 L
				TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);					//PA7	CH1 H 		GND -
			}
		}break;
		case MOTOR3:{
			if(Duty_t==0){
				TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Disable);					//PC7  	CH2 L
				TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);					//PB0  	CH2 H 		GND -
				
				TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Disable);					//PC8  	CH2 L
				TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);					//PB1  	CH2 H 		GND -
			} else if (Dir_t>0){
				TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_ForcedAction_Active);		//强制设置为高电平 
				TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Disable);					//PC7  	CH2 L
				TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);					//PB0  	CH2 H 		GND -
 				
				TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);	
				TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);					//PC8  	CH2 PWM  	VCC +
				TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCx_Disable);					//PB1  	CH2 L
				TIM_SetCompare3(TIM8, Duty_t);			
			} else {
				TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
				TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);					//PC7	CH2 PWM  	VCC +
				TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Disable);					//PB0	CH2 L
				TIM_SetCompare2(TIM8, Duty_t);
				
				TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_ForcedAction_Active);		//强制设置为高电平	
				TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Disable);					//PC8	CH3 L
				TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);					//PB1	CH3 H 		GND -				
			}
		}break;
		case MOTOR4:{
			if(Duty_t==0){
				TIM_SetCompare4(TIM1, 0);
				TIM_SetCompare4(TIM8, 0);
			} else if(Dir_t>0){
				TIM_SetCompare4(TIM1, 0);
				TIM_SetCompare4(TIM8, Duty_t);
			} else {
				TIM_SetCompare4(TIM1, Duty_t);
				TIM_SetCompare4(TIM8, 0);
			}
		}break;
	}
}
void MotorInit(uint8_t mName_t,uint32_t mPwm_Arr,uint32_t mPwm_Psc,uint8_t mMotorType){
	if(mMotorType == M_DCM_4WD){
		MC_PWMInit(mName_t,mPwm_Arr, mPwm_Psc);
		return;
	}
	GPIO_TypeDef* MOTOR_A_PORT[MOTORn] = {STARBOT_MOTOR1_A_GPIO_PORT, STARBOT_MOTOR2_A_GPIO_PORT, 
											STARBOT_MOTOR3_A_GPIO_PORT,STARBOT_MOTOR4_A_GPIO_PORT};
	GPIO_TypeDef* MOTOR_B_PORT[MOTORn] = {STARBOT_MOTOR1_B_GPIO_PORT, STARBOT_MOTOR2_B_GPIO_PORT, 
											STARBOT_MOTOR3_B_GPIO_PORT,STARBOT_MOTOR4_B_GPIO_PORT};
	TIM_TypeDef * MOTOR_TIM[MOTORn] = {STARBOT_MOTOR1_TIM, STARBOT_MOTOR2_TIM, \
											STARBOT_MOTOR3_TIM, STARBOT_MOTOR4_TIM};

	const uint32_t  MOTOR_TIM_CLK[MOTORn] = {STARBOT_MOTOR1_TIM_CLK, STARBOT_MOTOR2_TIM_CLK, \
												STARBOT_MOTOR3_TIM_CLK, STARBOT_MOTOR4_TIM_CLK};  
//	const uint16_t  MOTOR_A_CHANNEL[MOTORn] = {STARBOT_MOTOR1_A_CHANNEL, STARBOT_MOTOR2_A_CHANNEL, \
//												STARBOT_MOTOR3_A_CHANNEL, STARBOT_MOTOR4_A_CHANNEL};
	//const uint16_t  MOTOR_B_CHANNEL[MOTORn] = {STARBOT_MOTOR1_B_CHANNEL, STARBOT_MOTOR2_B_CHANNEL, \
	//											STARBOT_MOTOR3_B_CHANNEL, STARBOT_MOTOR4_B_CHANNEL};
	const uint16_t  MOTOR_A_PINSOU[MOTORn] = {STARBOT_MOTOR1_A_PINSOU,STARBOT_MOTOR2_A_PINSOU,\
												STARBOT_MOTOR3_A_PINSOU,STARBOT_MOTOR4_A_PINSOU};
	const uint16_t  MOTOR_B_PINSOU[MOTORn] = {STARBOT_MOTOR1_B_PINSOU,STARBOT_MOTOR2_B_PINSOU,\
												STARBOT_MOTOR3_B_PINSOU,STARBOT_MOTOR4_B_PINSOU};
	const uint32_t  MOTOR_A_PORT_CLK[MOTORn] = {STARBOT_MOTOR1_A_GPIO_CLK, STARBOT_MOTOR2_A_GPIO_CLK, \
												STARBOT_MOTOR3_A_GPIO_CLK, STARBOT_MOTOR4_A_GPIO_CLK};
	const uint32_t  MOTOR_B_PORT_CLK[MOTORn] = {STARBOT_MOTOR1_B_GPIO_CLK, STARBOT_MOTOR2_B_GPIO_CLK, \
												STARBOT_MOTOR3_B_GPIO_CLK, STARBOT_MOTOR4_B_GPIO_CLK};
	const uint32_t  MOTOR_AF_TIM[MOTORn] = {STARBOT_MOTOR1_AF_TIM,STARBOT_MOTOR2_AF_TIM,\
												STARBOT_MOTOR3_AF_TIM,STARBOT_MOTOR4_AF_TIM};
	const uint16_t  MOTOR_A_PIN[MOTORn] = {STARBOT_MOTOR1_A_PIN, STARBOT_MOTOR2_A_PIN, \
												STARBOT_MOTOR3_A_PIN, STARBOT_MOTOR4_A_PIN,};
	const uint16_t  MOTOR_B_PIN[MOTORn] = {STARBOT_MOTOR1_B_PIN, STARBOT_MOTOR2_B_PIN,\
												STARBOT_MOTOR3_B_PIN, STARBOT_MOTOR4_B_PIN};
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;
	TIM_BDTRInitTypeDef      TIM_BDTRInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(MOTOR_A_PORT_CLK[mName_t] | MOTOR_B_PORT_CLK[mName_t], ENABLE);
	RCC_APB2PeriphClockCmd(MOTOR_TIM_CLK[mName_t], ENABLE);
	switch(mMotorType){
		case M_A4950:{
				/** init motor_ gpio **/
			GPIO_InitStructure.GPIO_Pin   = MOTOR_A_PIN[mName_t];
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;							//复用功能
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;						//速度100MHz
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 							//推挽复用输出
			GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP;  
			GPIO_Init(MOTOR_A_PORT[mName_t], &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin = MOTOR_B_PIN[mName_t];
			GPIO_Init(MOTOR_B_PORT[mName_t], &GPIO_InitStructure);

			GPIO_PinAFConfig(MOTOR_A_PORT[mName_t],MOTOR_A_PINSOU[mName_t],MOTOR_AF_TIM[mName_t]);  //GPIOA8复用为定时器1
			GPIO_PinAFConfig(MOTOR_B_PORT[mName_t],MOTOR_B_PINSOU[mName_t],MOTOR_AF_TIM[mName_t]);  //GPIOA7复用为定时器1
			
			//init timx
			//pwm value ((1 + psc)/168M)*(1+arr)
			//eg: ((1+143)/168M)*(1+9999) = 0.02s --10000 count use 0.02s
			//set arduino pwm value 490hz 255 count 
			//((1 + 575)/168M)(1 + 254) = (1 / 490)
			//TIM_DeInit(MOTOR_TIM[mName_t]);
			TIM_TimeBaseInitStructure.TIM_Period = mPwm_Arr-1;      					//arr 自动重装载值
			TIM_TimeBaseInitStructure.TIM_Prescaler = mPwm_Psc-1;     					//psc 定时器分频
			TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
			TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;						//重复计数次数然后进入中断
			TIM_TimeBaseInit(MOTOR_TIM[mName_t], &TIM_TimeBaseInitStructure); 

			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                      		//选择定时器模式:TIM脉冲宽度调制模式2
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;          		//比较输出使能
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;       		//互补输出允许 TIM_OutputNState_Disable
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;              		//High为占空比高极性，此时占空比为50%，Low则为反极性，占空比为50%
			TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;		      		//指定空闲状态下的TIM输出比较的引脚状态。		
			switch(mName_t){
				case MOTOR1:{
					TIM_OC3Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);           
					TIM_OC3PreloadConfig(MOTOR_TIM[mName_t],TIM_OCPreload_Enable);

					TIM_OC2Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);
					TIM_OC2PreloadConfig(MOTOR_TIM[mName_t],TIM_OCPreload_Enable);
					TIM_SetCompare2(MOTOR_TIM[mName_t], 0);	
					TIM_SetCompare3(MOTOR_TIM[mName_t], 0);
				}break;
				case MOTOR2:{
					TIM_OC1Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);           
					TIM_OC1PreloadConfig(MOTOR_TIM[mName_t],TIM_OCPreload_Enable);

					TIM_OC4Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);
					TIM_OC4PreloadConfig(MOTOR_TIM[mName_t],TIM_OCPreload_Enable);
					TIM_SetCompare1(MOTOR_TIM[mName_t], 0);	
					TIM_SetCompare4(MOTOR_TIM[mName_t], 0);	
				}break;
				case MOTOR3:{
					TIM_OC1Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);           
					TIM_OC1PreloadConfig(MOTOR_TIM[mName_t],TIM_OCPreload_Enable);

					TIM_OC2Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);
					TIM_OC2PreloadConfig(MOTOR_TIM[mName_t],TIM_OCPreload_Enable);
					TIM_SetCompare1(MOTOR_TIM[mName_t], 0);	
					TIM_SetCompare2(MOTOR_TIM[mName_t], 0);	
				}break;
				case MOTOR4:{
					TIM_OC3Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);           
					TIM_OC3PreloadConfig(MOTOR_TIM[mName_t],TIM_OCPreload_Enable); 

					TIM_OC4Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);           
					TIM_OC4PreloadConfig(MOTOR_TIM[mName_t],TIM_OCPreload_Enable); 
					TIM_SetCompare3(MOTOR_TIM[mName_t], 0);	
					TIM_SetCompare4(MOTOR_TIM[mName_t], 0);	
				}break;
			}
			TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
			TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
			TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
			//bit7~5 = 111,则deadtime = (32 + (bit4~bit0)* 16*1/fosc)ns = (32+31)*16*1/72000000 = 14us
			////死区时间  72:1us 172:3us 205:5us		
			TIM_BDTRInitStructure.TIM_DeadTime = 0x94;
			TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;               
			TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
			TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
			TIM_BDTRConfig(MOTOR_TIM[mName_t],&TIM_BDTRInitStructure);
			
			TIM_ARRPreloadConfig(MOTOR_TIM[mName_t],ENABLE);
			TIM_Cmd(MOTOR_TIM[mName_t], ENABLE);
			TIM_CtrlPWMOutputs(MOTOR_TIM[mName_t], ENABLE);
		}break;
		case M_BTN797X:{
				// init motor rotation direction gpio 
			GPIO_InitStructure.GPIO_Pin = MOTOR_B_PIN[mName_t];	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;							//普通输出模式
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;							//推挽输出
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;						//100MHz
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;							//上拉
			GPIO_Init(MOTOR_B_PORT[mName_t], &GPIO_InitStructure);					//初始化GPIO
			GPIO_ResetBits(MOTOR_B_PORT[mName_t],MOTOR_B_PIN[mName_t]);
			//init pwm gpio 
			GPIO_InitStructure.GPIO_Pin   =  MOTOR_A_PIN[mName_t];
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;							//复用功能
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;						//速度100MHz
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 							//推挽复用输出
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init(MOTOR_A_PORT[mName_t], &GPIO_InitStructure);
			GPIO_PinAFConfig(MOTOR_A_PORT[mName_t],MOTOR_A_PINSOU[mName_t],MOTOR_AF_TIM[mName_t]);  //GPIOxx复用为定时器xx
			//init tim 
			TIM_TimeBaseInitStructure.TIM_Period = mPwm_Arr-1; 						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
			TIM_TimeBaseInitStructure.TIM_Prescaler =mPwm_Psc-1; 					//设置用来作为TIMx时钟频率除数的预分频值 
			TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2; 			//设置时钟分割:TDTS = Tck_tim
			TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 		//TIM向上计数模式
			TIM_TimeBaseInit(MOTOR_TIM[mName_t], &TIM_TimeBaseInitStructure); 	 	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

			//初始化TIM1 Channel2 PWM模式	 
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 						//选择定时器模式:TIM脉冲宽度调制模式2
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 			//比较输出使能
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 				//输出极性:TIM输出比较极性高
			TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
			switch(mName_t){
				case MOTOR1:{
					TIM_OC2Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);  			 //根据T指定的参数初始化外设TIMxx OC1
					TIM_OC2PreloadConfig(MOTOR_TIM[mName_t], TIM_OCPreload_Enable);  	 //使能TIMxx在CCR2上的预装载寄存器
					TIM_SetCompare2(MOTOR_TIM[mName_t], 0);	
					
		//			TIM_OC3Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);  			 //根据T指定的参数初始化外设TIMxx OC1
		//			TIM_OC3PreloadConfig(MOTOR_TIM[mName_t], TIM_OCPreload_Enable);  	 //使能TIMxx在CCR2上的预装载寄存器
		//			TIM_SetCompare3(MOTOR_TIM[mName_t], 0);
				}break;
				case MOTOR2:{
					TIM_OC1Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);  			 //根据T指定的参数初始化外设TIMxx OC2
					TIM_OC1PreloadConfig(MOTOR_TIM[mName_t], TIM_OCPreload_Enable);  	 //使能TIMxx在CCR2上的预装载寄存器
					TIM_SetCompare1(MOTOR_TIM[mName_t], 0);	
					
		//			TIM_OC4Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);  			 //根据T指定的参数初始化外设TIMxx OC2
		//			TIM_OC4PreloadConfig(MOTOR_TIM[mName_t], TIM_OCPreload_Enable);  	 //使能TIMxx在CCR2上的预装载寄存器
		//			TIM_SetCompare4(MOTOR_TIM[mName_t], 0);	
				}break;
				case MOTOR3:{
		//			TIM_OC1Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);  			 //根据T指定的参数初始化外设TIMxx OC2
		//			TIM_OC1PreloadConfig(MOTOR_TIM[mName_t], TIM_OCPreload_Enable);  	 //使能TIMxx在CCR2上的预装载寄存器
		//			TIM_SetCompare1(MOTOR_TIM[mName_t], 0);	
					
					TIM_OC2Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);  			 //根据T指定的参数初始化外设TIMxx OC2
					TIM_OC2PreloadConfig(MOTOR_TIM[mName_t], TIM_OCPreload_Enable);  	 //使能TIMxx在CCR2上的预装载寄存器
					TIM_SetCompare2(MOTOR_TIM[mName_t], 0);
				}break;
				case MOTOR4:{
					TIM_OC3Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);  			 //根据T指定的参数初始化外设TIMxx OC2
					TIM_OC3PreloadConfig(MOTOR_TIM[mName_t], TIM_OCPreload_Enable);  	 //使能TIMxx在CCR2上的预装载寄存器
					TIM_SetCompare3(MOTOR_TIM[mName_t], 0);
					
		//			TIM_OC4Init(MOTOR_TIM[mName_t], &TIM_OCInitStructure);  			 //根据T指定的参数初始化外设TIMxx OC2
		//			TIM_OC4PreloadConfig(MOTOR_TIM[mName_t], TIM_OCPreload_Enable);  	 //使能TIMxx在CCR2上的预装载寄存器
		//			TIM_SetCompare4(MOTOR_TIM[mName_t], 0);
				}break;
			}
			TIM_ARRPreloadConfig(MOTOR_TIM[mName_t],ENABLE);
			TIM_Cmd(MOTOR_TIM[mName_t], ENABLE);  								 	//使能TIMxx
			TIM_CtrlPWMOutputs(MOTOR_TIM[mName_t], ENABLE);                     	//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题 
		}break;
	}		
}
static void MC_PWMInit(uint8_t mName_t,uint32_t mPwm_Arr,uint32_t mPwm_Psc){
//	if(mName_t==MOTOR4){
//		ServoInit(SERVO1,mPwm_Arr,mPwm_Psc);
//		ServoInit(SERVO2,mPwm_Arr,mPwm_Psc);
//		return;
//	} 
	const uint8_t  HPwm_MxUpH_AF_TIM[MOTORn] 	= {HW_M1UpH_TIM_AF, HW_M2UpH_TIM_AF, HW_M3UpH_TIM_AF,HW_M4UpH_TIM_AF};
	const uint8_t  HPwm_MxUpL_AF_TIM[MOTORn] 	= {HW_M1UpL_TIM_AF, HW_M2UpL_TIM_AF, HW_M3UpL_TIM_AF};
	const uint8_t  HPwm_MxDowH_AF_TIM[MOTORn] 	= {HW_M1DowH_TIM_AF,HW_M2DowH_TIM_AF,HW_M3DowH_TIM_AF,HW_M4DowH_TIM_AF};
	const uint8_t  HPwm_MxDowL_AF_TIM[MOTORn] 	= {HW_M1DowL_TIM_AF,HW_M2DowL_TIM_AF,HW_M3DowL_TIM_AF};
	
	GPIO_TypeDef* HPwm_MxUpH_PORT[MOTORn] 		= {HW_M1UpH_GPIO, HW_M2UpH_GPIO, HW_M3UpH_GPIO, HW_M4UpH_GPIO};
	GPIO_TypeDef* HPwm_MxUpL_PORT[MOTORn] 		= {HW_M1UpL_GPIO, HW_M2UpL_GPIO, HW_M3UpL_GPIO};
	GPIO_TypeDef* HPwm_MxDowH_PORT[MOTORn] 		= {HW_M1DowH_GPIO,HW_M2DowH_GPIO,HW_M3DowH_GPIO,HW_M4DowH_GPIO};
	GPIO_TypeDef* HPwm_MxDowL_PORT[MOTORn] 		= {HW_M1DowL_GPIO,HW_M2DowL_GPIO,HW_M3DowL_GPIO};

	const uint16_t  HPwm_MxUpH_PIN[MOTORn] 		= {HW_M1UpH_PIN, HW_M2UpH_PIN, HW_M3UpH_PIN, HW_M4UpH_PIN};
	const uint16_t  HPwm_MxUpL_PIN[MOTORn] 		= {HW_M1UpL_PIN, HW_M2UpL_PIN, HW_M3UpL_PIN};
	const uint16_t  HPwm_MxDowH_PIN[MOTORn] 	= {HW_M1DowH_PIN,HW_M2DowH_PIN,HW_M3DowH_PIN,HW_M4DowH_PIN};
	const uint16_t  HPwm_MxDowL_PIN[MOTORn] 	= {HW_M1DowL_PIN,HW_M2DowL_PIN,HW_M3DowL_PIN};
	
	const uint8_t  HPwm_MxUpH_PINSOU[MOTORn]	= {HW_M1UpH_SOURCE, HW_M2UpH_SOURCE, HW_M3UpH_SOURCE, HW_M4UpH_SOURCE};
	const uint8_t  HPwm_MxUpL_PINSOU[MOTORn]	= {HW_M1UpL_SOURCE, HW_M2UpL_SOURCE, HW_M3UpL_SOURCE};
	const uint8_t  HPwm_MxDowH_PINSOU[MOTORn]	= {HW_M1DowH_SOURCE,HW_M2DowH_SOURCE,HW_M3DowH_SOURCE,HW_M4DowH_SOURCE};
	const uint8_t  HPwm_MxDowL_PINSOU[MOTORn]	= {HW_M1DowL_SOURCE,HW_M2DowL_SOURCE,HW_M3DowL_SOURCE};
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef PWM_TIM_TimeBaseStructure;
	TIM_OCInitTypeDef PWM_TIM_OCInitStructure;
	TIM_BDTRInitTypeDef PWM_TIM_BDTRInitStructure;
	
	TIM1->CNT = 0;TIM8->CNT = 0;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_TIM8, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = HPwm_MxUpH_PIN[mName_t];
	GPIO_Init(HPwm_MxUpH_PORT[mName_t], &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = HPwm_MxDowH_PIN[mName_t];
	GPIO_Init(HPwm_MxDowH_PORT[mName_t], &GPIO_InitStructure);
	
	GPIO_PinAFConfig(HPwm_MxUpH_PORT[mName_t], HPwm_MxUpH_PINSOU[mName_t], HPwm_MxUpH_AF_TIM[mName_t]);
	GPIO_PinAFConfig(HPwm_MxDowH_PORT[mName_t],HPwm_MxDowH_PINSOU[mName_t],HPwm_MxDowH_AF_TIM[mName_t]);
	
	if(mName_t != MOTOR4){
		GPIO_InitStructure.GPIO_Pin = HPwm_MxUpL_PIN[mName_t];
		GPIO_Init(HPwm_MxUpL_PORT[mName_t], &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = HPwm_MxDowL_PIN[mName_t];
		GPIO_Init(HPwm_MxDowL_PORT[mName_t], &GPIO_InitStructure);
		
		GPIO_PinAFConfig(HPwm_MxUpL_PORT[mName_t],HPwm_MxUpL_PINSOU[mName_t],HPwm_MxUpL_AF_TIM[mName_t]);
		GPIO_PinAFConfig(HPwm_MxDowL_PORT[mName_t],HPwm_MxDowL_PINSOU[mName_t],HPwm_MxDowL_AF_TIM[mName_t]);
	}
	TIM_TimeBaseStructInit(&PWM_TIM_TimeBaseStructure);
	PWM_TIM_TimeBaseStructure.TIM_Prescaler = mPwm_Psc;
	PWM_TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM_CounterMode_CenterAligned1;
	PWM_TIM_TimeBaseStructure.TIM_Period = mPwm_Arr;
	PWM_TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	PWM_TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &PWM_TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM8, &PWM_TIM_TimeBaseStructure);

	TIM_OCStructInit(&PWM_TIM_OCInitStructure);
	PWM_TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	PWM_TIM_OCInitStructure.TIM_OutputState =  TIM_OutputState_Enable;
	PWM_TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	PWM_TIM_OCInitStructure.TIM_Pulse = 0;//mPwm_Arr>>1;
	PWM_TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	PWM_TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	PWM_TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	PWM_TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	switch(mName_t){
		case MOTOR1:{
			TIM_OC1Init(TIM1, &PWM_TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
			TIM_OC2Init(TIM1, &PWM_TIM_OCInitStructure);
			TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		}break;
		case MOTOR2:{
			TIM_OC3Init(TIM1, &PWM_TIM_OCInitStructure);
			TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
			TIM_OC1Init(TIM8, &PWM_TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
		}break;
		case MOTOR3:{
			TIM_OC2Init(TIM8, &PWM_TIM_OCInitStructure);
			TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
			TIM_OC3Init(TIM8, &PWM_TIM_OCInitStructure);
			TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);	
		}break;
		case MOTOR4:{
			TIM_OC4Init(TIM1, &PWM_TIM_OCInitStructure);
			TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
			TIM_OC4Init(TIM8, &PWM_TIM_OCInitStructure);
			TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);	
		}break;
	}
	PWM_TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	PWM_TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	PWM_TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; 
	PWM_TIM_BDTRInitStructure.TIM_DeadTime = 120;
	PWM_TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	PWM_TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	PWM_TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	TIM_BDTRConfig(TIM1, &PWM_TIM_BDTRInitStructure);
	TIM_BDTRConfig(TIM8, &PWM_TIM_BDTRInitStructure);
	
	TIM_Cmd(TIM1, ENABLE);TIM_Cmd(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);TIM_CtrlPWMOutputs(TIM8, ENABLE);
}
void EncoderInit(uint8_t eName_t,uint8_t mMotorType){
	GPIO_TypeDef* ENCODER_A_PORT[ENCODERn] = {STARBOT_ENCODER1_A_GPIO_PORT, STARBOT_ENCODER2_A_GPIO_PORT, STARBOT_ENCODER3_A_GPIO_PORT, STARBOT_ENCODER4_A_GPIO_PORT};
	GPIO_TypeDef* ENCODER_B_PORT[ENCODERn] = {STARBOT_ENCODER1_B_GPIO_PORT, STARBOT_ENCODER2_B_GPIO_PORT, STARBOT_ENCODER3_B_GPIO_PORT, STARBOT_ENCODER4_B_GPIO_PORT};
	TIM_TypeDef*  ENCODER_TIM[ENCODERn] = {STARBOT_ENCODER1_TIM, STARBOT_ENCODER2_TIM, STARBOT_ENCODER3_TIM, STARBOT_ENCODER4_TIM,};
	const uint16_t  ENCODER_TIR[ENCODERn] = {STARBOT_ENCODER1_TIR,STARBOT_ENCODER2_TIR,STARBOT_ENCODER3_TIR,STARBOT_ENCODER4_TIR};
	const uint32_t  ENCODER_TIM_CLK[ENCODERn] = {STARBOT_ENCODER1_TIM_CLK, STARBOT_ENCODER2_TIM_CLK, STARBOT_ENCODER3_TIM_CLK, STARBOT_ENCODER4_TIM_CLK};
	const uint16_t  ENCODER_GPIO_AF_TIM[ENCODERn] = {STARBOT_ENCODER1_GPIO_AF_TIM,STARBOT_ENCODER2_GPIO_AF_TIM,STARBOT_ENCODER3_GPIO_AF_TIM,STARBOT_ENCODER4_GPIO_AF_TIM};
	uint32_t  ENCODER_A_PORT_CLK[ENCODERn] = {STARBOT_ENCODER1_A_GPIO_CLK, STARBOT_ENCODER2_A_GPIO_CLK, STARBOT_ENCODER3_A_GPIO_CLK, STARBOT_ENCODER4_A_GPIO_CLK,};
	uint32_t  ENCODER_B_PORT_CLK[ENCODERn] = {STARBOT_ENCODER1_B_GPIO_CLK,STARBOT_ENCODER2_B_GPIO_CLK,STARBOT_ENCODER3_B_GPIO_CLK,STARBOT_ENCODER4_B_GPIO_CLK};
	uint16_t  ENCODER_A_PIN[ENCODERn] = {STARBOT_ENCODER1_A_PIN, STARBOT_ENCODER2_A_PIN, STARBOT_ENCODER3_A_PIN, STARBOT_ENCODER4_A_PIN,};
	uint16_t  ENCODER_B_PIN[ENCODERn] = {STARBOT_ENCODER1_B_PIN, STARBOT_ENCODER2_B_PIN, STARBOT_ENCODER3_B_PIN, STARBOT_ENCODER4_B_PIN};
	uint16_t  ENCODER_A_GPIO_PinSource[ENCODERn] ={STARBOT_ENCODER1_A_GPIO_PinSource,STARBOT_ENCODER2_A_GPIO_PinSource,STARBOT_ENCODER3_A_GPIO_PinSource,STARBOT_ENCODER4_A_GPIO_PinSource};
	uint16_t  ENCODER_B_GPIO_PinSource[ENCODERn] ={STARBOT_ENCODER1_B_GPIO_PinSource,STARBOT_ENCODER2_B_GPIO_PinSource,STARBOT_ENCODER3_B_GPIO_PinSource,STARBOT_ENCODER4_B_GPIO_PinSource};

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	if(M_DCM_4WD == mMotorType && ENCODER3 == eName_t){
		ENCODER_A_PORT_CLK[eName_t] = STARBOT_ENCODER3_A2_GPIO_CLK;
		ENCODER_B_PORT_CLK[eName_t] = STARBOT_ENCODER3_B2_GPIO_CLK;
		ENCODER_A_PORT[eName_t] = STARBOT_ENCODER3_A2_GPIO_PORT;
		ENCODER_B_PORT[eName_t] = STARBOT_ENCODER3_B2_GPIO_PORT;
		ENCODER_A_PIN[eName_t] = STARBOT_ENCODER3_A2_PIN;
		ENCODER_B_PIN[eName_t] = STARBOT_ENCODER3_B2_PIN;
		ENCODER_A_GPIO_PinSource[eName_t] = STARBOT_ENCODER3_A2_GPIO_PinSource;
		ENCODER_B_GPIO_PinSource[eName_t] = STARBOT_ENCODER3_B2_GPIO_PinSource;
	}
	RCC_APB1PeriphClockCmd(ENCODER_TIM_CLK[eName_t], ENABLE);  
	RCC_AHB1PeriphClockCmd(ENCODER_A_PORT_CLK[eName_t]|ENCODER_B_PORT_CLK[eName_t], ENABLE);

	GPIO_InitStructure.GPIO_Pin = ENCODER_A_PIN[eName_t];
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(ENCODER_A_PORT[eName_t], &GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin = ENCODER_B_PIN[eName_t]; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(ENCODER_B_PORT[eName_t], &GPIO_InitStructure);  

	GPIO_PinAFConfig(ENCODER_A_PORT[eName_t],ENCODER_A_GPIO_PinSource[eName_t],ENCODER_GPIO_AF_TIM[eName_t]);  
	GPIO_PinAFConfig(ENCODER_B_PORT[eName_t],ENCODER_B_GPIO_PinSource[eName_t],ENCODER_GPIO_AF_TIM[eName_t]);   

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 																					// No prescaling //不分频
	if(ENCODER1 == eName_t || ENCODER2 == eName_t){
		TIM_TimeBaseStructure.TIM_Period = 0xffffffff;  																		//设定计数器自动重装值
	} else {
		TIM_TimeBaseStructure.TIM_Period = 0xffff;  																			//设定计数器自动重装值
	}

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 																	//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 																//TIM向上计数    
	TIM_TimeBaseInit(ENCODER_TIM[eName_t], &TIM_TimeBaseStructure);  															//初始化定时器
	TIM_EncoderInterfaceConfig(ENCODER_TIM[eName_t], TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);		//使用编码器模式3

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(ENCODER_TIM[eName_t], &TIM_ICInitStructure);
	TIM_SetCounter(ENCODER_TIM[eName_t],0);
	TIM_Cmd(ENCODER_TIM[eName_t], ENABLE);
}

void ServoInit(uint8_t sName_t,uint32_t mPwm_Arr,uint32_t mPwm_Psc){
	GPIO_TypeDef* SERVO_PORT[SERVOn] = {STARBOT_SERVO1_GPIO_PORT, STARBOT_SERVO2_GPIO_PORT,STARBOT_SERVO3_GPIO_PORT,STARBOT_SERVO4_GPIO_PORT};
	TIM_TypeDef*  SERVO_TIM[SERVOn] = {STARBOT_SERVO1_TIM, STARBOT_SERVO2_TIM,STARBOT_SERVO3_TIM,STARBOT_SERVO4_TIM};
	const uint32_t  SERVO_PORT_CLK[SERVOn] = {STARBOT_SERVO1_GPIO_CLK, STARBOT_SERVO2_GPIO_CLK,STARBOT_SERVO3_GPIO_CLK,STARBOT_SERVO4_GPIO_CLK};
	const uint16_t  SERVO_PIN[SERVOn] = {STARBOT_SERVO1_PIN, STARBOT_SERVO2_PIN,STARBOT_SERVO3_PIN,STARBOT_SERVO4_PIN};
	const uint32_t  SERVO_TIM_CLK[SERVOn] = {STARBOT_SERVO1_TIM_CLK, STARBOT_SERVO2_TIM_CLK,STARBOT_SERVO3_TIM_CLK,STARBOT_SERVO4_TIM_CLK};
	const uint32_t  SERVO_AF_TIM[SERVOn] = {STARBOT_SERVO1_AFTIM, STARBOT_SERVO2_AFTIM,STARBOT_SERVO3_AFTIM,STARBOT_SERVO4_AFTIM};
	const uint32_t  SERVO_SOU_PIN[SERVOn] = {STARBOT_SERVO1_SOUPIN, STARBOT_SERVO2_SOUPIN,STARBOT_SERVO3_SOUPIN,STARBOT_SERVO4_SOUPIN};
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_AHB1PeriphClockCmd(SERVO_PORT_CLK[sName_t], ENABLE);
	if(sName_t == SERVO1 || sName_t == SERVO2){
		RCC_APB2PeriphClockCmd(SERVO_TIM_CLK[sName_t], ENABLE);
	}else{
		RCC_APB1PeriphClockCmd(SERVO_TIM_CLK[sName_t], ENABLE);
	}
	GPIO_PinAFConfig(SERVO_PORT[sName_t],SERVO_SOU_PIN[sName_t],SERVO_AF_TIM[sName_t]); //GPIO复用为定时器X输出
	GPIO_InitStructure.GPIO_Pin     = SERVO_PIN[sName_t];
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;										//复用功能
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;								//速度100MHz
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP; 									//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_DOWN; 									//下拉
	GPIO_Init(SERVO_PORT[sName_t], &GPIO_InitStructure);
	if(mPwm_Arr == 0){
		mPwm_Arr = 19999;
	}
	if(mPwm_Psc == 0){
		if(sName_t == SERVO1 || sName_t ==SERVO2){
			mPwm_Psc = 167;
		}else{
			mPwm_Psc=83;
		}
	}
	TIM_BaseInitStructure.TIM_Period                = mPwm_Arr;
	TIM_BaseInitStructure.TIM_Prescaler         	= mPwm_Psc;
	TIM_BaseInitStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
	TIM_BaseInitStructure.TIM_CounterMode           = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter     = 0;

	TIM_TimeBaseInit(SERVO_TIM[sName_t], &TIM_BaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 

	if(sName_t == SERVO2 || sName_t ==SERVO3){
		TIM_OC1Init(SERVO_TIM[sName_t], &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(SERVO_TIM[sName_t], TIM_OCPreload_Enable);
	}

	if(sName_t == SERVO1 || sName_t ==SERVO4){
		TIM_OC2Init(SERVO_TIM[sName_t], &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(SERVO_TIM[sName_t], TIM_OCPreload_Enable);
	}

	TIM_ARRPreloadConfig(SERVO_TIM[sName_t], ENABLE);

//	TIM_CtrlPWMOutputs(SERVO_TIM[sName_t], ENABLE);
	TIM_Cmd(SERVO_TIM[sName_t], ENABLE);
}
void Ws2812TimeInit(uint8_t LedName){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(STARBOT_WS2812_2_GPIO_CLK, ENABLE);	
	RCC_APB1PeriphClockCmd(STARBOT_WS2812_2_TIM_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(STARBOT_WS2812_2_DMA_CLK, ENABLE);
	
	//初始化灯环RGB灯
	GPIO_InitStructure.GPIO_Pin = STARBOT_WS2812_1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(STARBOT_WS2812_1_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = STARBOT_WS2812_2_PIN;
	GPIO_Init(STARBOT_WS2812_2_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(STARBOT_WS2812_1_GPIO_PORT, STARBOT_WS2812_1_PINSOU, STARBOT_WS2812_1_AF_TIM);
	GPIO_PinAFConfig(STARBOT_WS2812_2_GPIO_PORT, STARBOT_WS2812_2_PINSOU, STARBOT_WS2812_2_AF_TIM);

	
	TIM_TimeBaseStructure.TIM_Period = 119; //800KHz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(STARBOT_WS2812_2_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(STARBOT_WS2812_1_TIM, &TIM_OCInitStructure);	 
	TIM_OC1PreloadConfig(STARBOT_WS2812_1_TIM, TIM_OCPreload_Enable);
	
	TIM_OC2Init(STARBOT_WS2812_2_TIM, &TIM_OCInitStructure);	 
	TIM_OC2PreloadConfig(STARBOT_WS2812_2_TIM, TIM_OCPreload_Enable);

	DMA_DeInit(STARBOT_WS2812_1_DMA_Stream);
	DMA_InitStructure.DMA_Channel = STARBOT_WS2812_1_DMA_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&STARBOT_WS2812_2_TIM->CCR1;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;	

	DMA_Init(STARBOT_WS2812_1_DMA_Stream, &DMA_InitStructure);
	TIM_DMACmd(STARBOT_WS2812_1_TIM, STARBOT_WS2812_1_TIM_DMA_CC, ENABLE);			//使能TIMx CC1 DMA

	DMA_DeInit(STARBOT_WS2812_2_DMA_Stream);
	DMA_InitStructure.DMA_Channel = STARBOT_WS2812_2_DMA_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&STARBOT_WS2812_2_TIM->CCR2;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;	

	DMA_Init(STARBOT_WS2812_2_DMA_Stream, &DMA_InitStructure);
	TIM_DMACmd(STARBOT_WS2812_2_TIM, STARBOT_WS2812_2_TIM_DMA_CC, ENABLE);			//使能TIMx CC2 DMA
}
void Ws2812SendBuffDma(uint8_t LedName,uint16_t *sendBuff,uint16_t buffLen){
	switch(LedName){
		case 0x01:{
			STARBOT_WS2812_1_DMA_Stream->M0AR = (uint32_t)sendBuff; 											//设置拷贝的数据地址 set memory addr
			STARBOT_WS2812_1_DMA_Stream->NDTR = buffLen;														//设置拷贝的长度
			TIM_Cmd(STARBOT_WS2812_1_TIM, ENABLE);
			TIM_DMACmd(STARBOT_WS2812_1_TIM, STARBOT_WS2812_1_TIM_DMA_CC, ENABLE);
			DMA_Cmd(STARBOT_WS2812_1_DMA_Stream, ENABLE);
			while(!DMA_GetFlagStatus(STARBOT_WS2812_1_DMA_Stream, STARBOT_WS2812_1_DMA_TCIF));
			DMA_Cmd(STARBOT_WS2812_1_DMA_Stream, DISABLE);
			DMA_ClearFlag(STARBOT_WS2812_1_DMA_Stream, STARBOT_WS2812_1_DMA_TCIF);
			TIM_Cmd(STARBOT_WS2812_1_TIM, DISABLE);	
		}break;
		case 0x02:{
			STARBOT_WS2812_2_DMA_Stream->M0AR = (uint32_t)sendBuff; 											//设置拷贝的数据地址 set memory addr
			STARBOT_WS2812_2_DMA_Stream->NDTR = buffLen;														//设置拷贝的长度													//设置拷贝的长度
			TIM_Cmd(STARBOT_WS2812_2_TIM, ENABLE);
			TIM_DMACmd(STARBOT_WS2812_2_TIM, STARBOT_WS2812_2_TIM_DMA_CC, ENABLE);
			DMA_Cmd(STARBOT_WS2812_2_DMA_Stream, ENABLE);
			while(!DMA_GetFlagStatus(STARBOT_WS2812_2_DMA_Stream, STARBOT_WS2812_2_DMA_TCIF));
			DMA_Cmd(STARBOT_WS2812_2_DMA_Stream, DISABLE);
			DMA_ClearFlag(STARBOT_WS2812_2_DMA_Stream, STARBOT_WS2812_2_DMA_TCIF);
			TIM_Cmd(STARBOT_WS2812_2_TIM, DISABLE);
		}break;
		case 0x03:{
//			DMA_Cmd(STARBOT_WS2812_1_DMA_Stream,DISABLE); 														//禁用DMA Disable DMA
			STARBOT_WS2812_1_DMA_Stream->M0AR = (uint32_t)sendBuff; 											//设置拷贝的数据地址 set memory addr
			STARBOT_WS2812_1_DMA_Stream->NDTR = buffLen;														//设置拷贝的长度
			TIM_Cmd(STARBOT_WS2812_1_TIM, ENABLE);
			TIM_DMACmd(STARBOT_WS2812_1_TIM, STARBOT_WS2812_1_TIM_DMA_CC, ENABLE);
			DMA_Cmd(STARBOT_WS2812_1_DMA_Stream, ENABLE);
			while(!DMA_GetFlagStatus(STARBOT_WS2812_1_DMA_Stream, STARBOT_WS2812_1_DMA_TCIF));
			DMA_Cmd(STARBOT_WS2812_1_DMA_Stream, DISABLE);
			DMA_ClearFlag(STARBOT_WS2812_1_DMA_Stream, STARBOT_WS2812_1_DMA_TCIF);
			TIM_Cmd(STARBOT_WS2812_1_TIM, DISABLE);

//			DMA_Cmd(STARBOT_WS2812_2_DMA_Stream,DISABLE); 														//禁用DMA Disable DMA
			STARBOT_WS2812_2_DMA_Stream->M0AR = (uint32_t)sendBuff; 											//设置拷贝的数据地址 set memory addr
			STARBOT_WS2812_2_DMA_Stream->NDTR = buffLen;														//设置拷贝的长度													//设置拷贝的长度
			TIM_Cmd(STARBOT_WS2812_2_TIM, ENABLE);
			TIM_DMACmd(STARBOT_WS2812_2_TIM, STARBOT_WS2812_2_TIM_DMA_CC, ENABLE);
			DMA_Cmd(STARBOT_WS2812_2_DMA_Stream, ENABLE);
			while(!DMA_GetFlagStatus(STARBOT_WS2812_2_DMA_Stream, STARBOT_WS2812_2_DMA_TCIF));
			DMA_Cmd(STARBOT_WS2812_2_DMA_Stream, DISABLE);
			DMA_ClearFlag(STARBOT_WS2812_2_DMA_Stream, STARBOT_WS2812_2_DMA_TCIF);
			TIM_Cmd(STARBOT_WS2812_2_TIM, DISABLE);
		}break;
		case 0x07:{
		}break;
		case 0x0F:{
		}break;
		default:break;
	}
}
void getUARTConfigParam(const appInterfaceParam *inData,void *outPut){	
		USART_InitTypeDef *p_t = (USART_InitTypeDef *)outPut;
		p_t->USART_BaudRate = inData->Port_BaudRate;								//波特率设置
		switch(inData->Port_WordLength){
			case 0:
				p_t->USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
				break;
			case 1:
				p_t->USART_WordLength = USART_WordLength_9b;						//字长为9位数据格式
				break;
		}
		switch(inData->Port_WordLength){
			case 5:
				p_t->USART_StopBits = USART_StopBits_0_5;							//0.5停止位 
				break;
			case 10:
				p_t->USART_StopBits = USART_StopBits_1; 							//1停止位 
				break;
			case 15:
				p_t->USART_StopBits = USART_StopBits_1_5; 							//1.5停止位 
				break;
			case 20:
				p_t->USART_StopBits = USART_StopBits_2;								//2停止位 
				break;
		}
		switch(inData->Port_Parity){
			case 0:
				p_t->USART_Parity = USART_Parity_No;								//无奇偶校验位
				break;
			case 1:
				p_t->USART_Parity = USART_Parity_Even;								//偶校验位
				break;
			case 2:
				p_t->USART_Parity = USART_Parity_Odd;								//奇校验位
				break;
		}
		switch(inData->Port_Mode){
			case 0:
				p_t->USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式
				break;
			case 1:
				p_t->USART_Mode = USART_Mode_Rx;									//收模式
				break;
			case 2:
				p_t->USART_Mode = USART_Mode_Tx;									//发模式
				break;
		}
		switch(inData->Port_HFC){
			case 0:
				p_t->USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
				break;
			case 1:
				p_t->USART_HardwareFlowControl = USART_HardwareFlowControl_RTS;		//RTS硬件数据流控制
				break;		
			case 2:
				p_t->USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;		//CTS硬件数据流控制
				break;	
			case 3:
				p_t->USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;	//CTS RTS 硬件数据流控制
				break;				
		}
}
void dec_ppmInit(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

//	memset(&srcValue_t,0,sizeof(ppmStruct));
	
	RCC_APB1PeriphClockCmd(DEC_PPM_TIM_CLK,ENABLE);  
	RCC_AHB1PeriphClockCmd(DEC_PPM_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = DEC_PPM_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(DEC_PPM_PORT,&GPIO_InitStructure);

	GPIO_PinAFConfig(DEC_PPM_PORT,DEC_PPM_PINSOURCE,DEC_PPM_AF_TIM);
  
	TIM_TimeBaseStructure.TIM_Prescaler=DEC_PPM_TIM_PSC;
	TIM_TimeBaseStructure.TIM_Period=DEC_PPM_TIM_ARR;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(DEC_PPM_TIM,&TIM_TimeBaseStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 										//CC1S=01 	选择输入端 IC1映射到TI1上
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;								//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 						//映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(DEC_PPM_TIM, &TIM_ICInitStructure);

	TIM_ITConfig(DEC_PPM_TIM,TIM_IT_Update|TIM_IT_CC2,ENABLE);								//允许更新中断 ,允许CC1IE捕获中断	

	TIM_Cmd(DEC_PPM_TIM,ENABLE );

	NVIC_InitStructure.NVIC_IRQChannel = DEC_PPM_TIM_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void dec_dutyInit(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(DEC_DUTY_TIM_CLK,ENABLE);  
	RCC_AHB1PeriphClockCmd(DEC_DUTY_CH1_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(DEC_DUTY_CH2_CLK, ENABLE);
	
//	memset(&dutyValue_t,0,sizeof(dutyStruct));
	
	GPIO_InitStructure.GPIO_Pin = DEC_DUTY_CH1_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(DEC_DUTY_CH1_PORT,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = DEC_DUTY_CH2_PIN; 
	GPIO_Init(DEC_DUTY_CH2_PORT,&GPIO_InitStructure);

	GPIO_PinAFConfig(DEC_DUTY_CH1_PORT,DEC_DUTY_CH1_PINSOURCE,DEC_DUTY_AF_TIM);
	GPIO_PinAFConfig(DEC_DUTY_CH2_PORT,DEC_DUTY_CH2_PINSOURCE,DEC_DUTY_AF_TIM);
  
	TIM_TimeBaseStructure.TIM_Prescaler=DEC_DUTY_TIM_PSC;
	TIM_TimeBaseStructure.TIM_Period=DEC_DUTY_TIM_ARR;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(DEC_DUTY_TIM,&TIM_TimeBaseStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 										//CC1S=01 	选择输入端 IC1映射到TI1上
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;								//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 						//映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(DEC_DUTY_TIM, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 										//CC1S=01 	选择输入端 IC1映射到TI1上
	TIM_ICInit(DEC_DUTY_TIM, &TIM_ICInitStructure);

	TIM_ITConfig(DEC_DUTY_TIM,TIM_IT_CC1|TIM_IT_CC2,ENABLE);								//允许更新中断 ,允许CC1IE捕获中断	

	TIM_Cmd(DEC_DUTY_TIM,ENABLE );

	NVIC_InitStructure.NVIC_IRQChannel = DEC_DUTY_TIM_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
#endif
#ifdef __cplusplus
}
#endif












