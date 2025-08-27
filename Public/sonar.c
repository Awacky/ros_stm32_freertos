#ifdef __cplusplus
extern "C" {
#endif

#include "sonar.h"
typedef struct
{
	uint8_t Index;
	uint8_t CapStatus;
	uint16_t value[5];
	float   Old_value;
	float   Cal_value;
}SonarMag;
SonarMag SonarMag_1;
SonarMag SonarMag_2;
static uint8_t SonarId = 0;
Hw_GPIO_TypeDef* SONAR_TRIG_PORT[SONARn] = {STARBOT_SONAR1_TRIG_PORT,STARBOT_SONAR2_TRIG_PORT};
Hw_GPIO_TypeDef* SONAR_ECHO_PORT[SONARn] = {STARBOT_SONAR1_ECHO_PORT,STARBOT_SONAR2_ECHO_PORT};
Hw_TIM_TypeDef*  SONAR_TIM[SONARn]		 = {STARBOT_SONAR1_TIM,STARBOT_SONAR2_TIM};

const uint32_t SONAR_TRIG_CLK[SONARn]	 = {STARBOT_SONAR1_TRIG_CLK,STARBOT_SONAR2_TRIG_CLK};
const uint32_t SONAR_ECHO_CLK[SONARn]	 = {STARBOT_SONAR1_ECHO_CLK,STARBOT_SONAR2_ECHO_CLK};
const uint32_t SONAR_TIM_CLK[SONARn]	 = {STARBOT_SONAR1_TIM_CLK,STARBOT_SONAR2_TIM_CLK};

const uint16_t SONAR_TRIG_PIN[SONARn]	 = {STARBOT_SONAR1_TRIG_PIN,STARBOT_SONAR2_TRIG_PIN};
const uint16_t SONAR_ECHO_PIN[SONARn]	 = {STARBOT_SONAR1_ECHO_PIN,STARBOT_SONAR2_ECHO_PIN};
#ifndef STM32F10X_HD
const uint16_t SONAR_TIM_IRQ[SONARn]	 = {STARBOT_SONAR1_TIM_IRQ,STARBOT_SONAR2_TIM_IRQ};
#endif

#ifdef STM32F40_41xxx
const uint16_t SONAR_AF_TIM[SONARn]	= {STARBOT_SONAR1_AF_TIM,STARBOT_SONAR2_AF_TIM};
const uint16_t SONAR_PINSOURCE[SONARn] = {STARBOT_SONAR1_PINSOURCE,STARBOT_SONAR2_PINSOURCE};
#endif

void HwSonar_Init(uint16_t arr,uint16_t psc,uint8_t sName)
{
	#ifdef STM32F40_41xxx
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_ICInitTypeDef  TIM_ICInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB2PeriphClockCmd(SONAR_TIM_CLK[sName],ENABLE);  
		RCC_AHB1PeriphClockCmd(SONAR_ECHO_CLK[sName], ENABLE);
		RCC_AHB1PeriphClockCmd(SONAR_TRIG_CLK[sName], ENABLE);
		
		GPIO_InitStructure.GPIO_Pin = SONAR_ECHO_PIN[sName]; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(SONAR_ECHO_PORT[sName],&GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = SONAR_TRIG_PIN[sName]; 	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(SONAR_TRIG_PORT[sName], &GPIO_InitStructure);
		GPIO_PinAFConfig(SONAR_ECHO_PORT[sName],SONAR_PINSOURCE[sName],SONAR_AF_TIM[sName]);
	  
		TIM_TimeBaseStructure.TIM_Prescaler=psc;
		TIM_TimeBaseStructure.TIM_Period=arr-1;
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
		TIM_TimeBaseInit(SONAR_TIM[sName],&TIM_TimeBaseStructure);
		
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 										//CC1S=01 	选择输入端 IC1映射到TI1上
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;								//上升沿捕获
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 						//映射到TI1上
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x00;
		TIM_ICInit(SONAR_TIM[sName], &TIM_ICInitStructure);

		TIM_ITConfig(SONAR_TIM[sName],TIM_IT_Update|TIM_IT_CC1,ENABLE);							//允许更新中断 ,允许CC1IE捕获中断	

		TIM_Cmd(SONAR_TIM[sName],ENABLE );

		NVIC_InitStructure.NVIC_IRQChannel = SONAR_TIM_IRQ[sName];
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	#elif AT32F40x
	{
		gpio_init_type  gpio_init_struct = {0};
		tmr_input_config_type  tmr_input_config_struct;
		crm_periph_clock_enable(SONAR_TIM_CLK[sName], TRUE);
		crm_periph_clock_enable(SONAR_ECHO_CLK[sName], TRUE);
		crm_periph_clock_enable(SONAR_TRIG_CLK[sName], TRUE);

		gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
		gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
		gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
		gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
		gpio_init_struct.gpio_pins = SONAR_ECHO_PIN[sName];
		gpio_init(SONAR_ECHO_PORT[sName], &gpio_init_struct);
		
		gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
		gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
		gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
		gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;			
		gpio_init_struct.gpio_pins = SONAR_TRIG_PIN[sName];
		gpio_init(SONAR_TRIG_PORT[sName], &gpio_init_struct);
		
		  /* tmr3 counter mode configuration */
		tmr_base_init(SONAR_TIM[sName],arr,psc);
		tmr_cnt_dir_set(SONAR_TIM[sName], TMR_COUNT_UP);

		/* configure tmr3 channel2 to get clock signal */
		tmr_input_config_struct.input_channel_select = TMR_SELECT_CHANNEL_1;
		tmr_input_config_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
		tmr_input_config_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;							//上升沿捕获
		tmr_input_channel_init(SONAR_TIM[sName], &tmr_input_config_struct, TMR_CHANNEL_INPUT_DIV_1);

		tmr_interrupt_enable(SONAR_TIM[sName], TMR_C1_INT, TRUE);
		tmr_interrupt_enable(SONAR_TIM[sName], TMR_OVF_INT, TRUE);

		/* tmr2 trigger interrupt nvic init */
		nvic_irq_enable(SONAR_TIM_IRQ[sName], 0, 0);

		/* enable tmr3 */
		tmr_counter_enable(SONAR_TIM[sName], TRUE);
	}
	#endif
}

void Sonar_Init(uint8_t SonarID)
{	
	HwSonar_Init(STARBOT_SONAR1_TIM_ARR,STARBOT_SONAR1_TIM_PSC,SonarID);
	switch(SonarID){
		case Sonar1:{
			memset(&SonarMag_1,0,sizeof(SonarMag));
			SonarId |= 1;
		}break;
		case Sonar2:{
			memset(&SonarMag_2,0,sizeof(SonarMag));
			SonarId |= 2;
		}break;
	}
}
void Sonar1_TRIG_Enabled(void)
{	
	STARBOT_SONAR1_TRIG_ON();
	delay_us(50);
	STARBOT_SONAR1_TRIG_OFF();	
}
void Sonar2_TRIG_Enabled(void)
{	
	STARBOT_SONAR2_TRIG_ON();
    delay_us(50);
	STARBOT_SONAR2_TRIG_OFF();
}
void Sonar1_IRQHandler(void)														//TIM11IRQ
{ 	
	#ifdef STM32F40_41xxx
	{	
		if(TIM_GetITStatus(STARBOT_SONAR1_TIM, TIM_IT_Update) != RESET){	
			if(SonarMag_1.CapStatus){								
				SonarMag_1.CapStatus=0;
				TIM_Cmd(STARBOT_SONAR1_TIM,DISABLE ); 										//关闭定时器
				TIM_SetCounter(STARBOT_SONAR1_TIM,0);
				TIM_OC1PolarityConfig(STARBOT_SONAR1_TIM,TIM_ICPolarity_Rising); 			//CC1P=0 设置为上升沿捕获
				TIM_Cmd(STARBOT_SONAR1_TIM,ENABLE );										//使能定时器					
			} 
			TIM_ClearITPendingBit(STARBOT_SONAR1_TIM,TIM_IT_Update);
		}
		if(TIM_GetITStatus(STARBOT_SONAR1_TIM, TIM_IT_CC1) != RESET){
			if(SonarMag_1.CapStatus){														//捕获到低电平，则完成一次高电平时间获取							
				SonarMag_1.value[SonarMag_1.Index]  = TIM_GetCapture1(STARBOT_SONAR1_TIM);	//获取当前的捕获值.
				SonarMag_1.Index++;
				if(SonarMag_1.Index>2){
					SonarMag_1.Cal_value = 0;
					for(int i=0;i<3;i++){
						SonarMag_1.Cal_value += SonarMag_1.value[i];
					}
					SonarMag_1.Old_value = SonarMag_1.Cal_value/3.0f;
					SonarMag_1.Index = 0;
				}
				SonarMag_1.CapStatus = 0;
				TIM_Cmd(STARBOT_SONAR1_TIM,DISABLE ); 								//关闭定时器
				TIM_SetCounter(STARBOT_SONAR1_TIM,0);
				TIM_OC1PolarityConfig(STARBOT_SONAR1_TIM,TIM_ICPolarity_Rising); 	//CC1P=0 设置为上升沿捕获
				TIM_Cmd(STARBOT_SONAR1_TIM,ENABLE );								//使能定时器
			}else{																	//捕获到高电平，开始计算高电平捕获时间		
				SonarMag_1.CapStatus = 1;		
				TIM_Cmd(STARBOT_SONAR1_TIM,DISABLE ); 								//关闭定时器 	   
				TIM_SetCounter(STARBOT_SONAR1_TIM,0);				
				TIM_OC1PolarityConfig(STARBOT_SONAR1_TIM,TIM_ICPolarity_Falling);	//CC1P=1 设置为下降沿捕获
				TIM_Cmd(STARBOT_SONAR1_TIM,ENABLE );								//使能定时器
			}	
			TIM_ClearITPendingBit(STARBOT_SONAR1_TIM, TIM_IT_CC1);			
		}			     	    					   
	}		
	#elif AT32F40x
	{
		if(tmr_flag_get(STARBOT_SONAR1_TIM, TMR_OVF_FLAG) != RESET)
		{	 
			if(SonarMag_1.CapStatus){								
				SonarMag_1.CapStatus=0;
				STARBOT_SONAR1_TIM->ctrl1_bit.tmren = FALSE;
				STARBOT_SONAR1_TIM->cval = 0;	
				STARBOT_SONAR1_TIM->cctrl_bit.c1p        = (uint32_t)TMR_INPUT_RISING_EDGE;		//CC1P=0 设置为上升沿捕获
				STARBOT_SONAR1_TIM->ctrl1_bit.tmren = TRUE;										//使能定时器					
			} 
			tmr_flag_clear(STARBOT_SONAR1_TIM, TMR_OVF_FLAG);
		}
		if(tmr_flag_get(STARBOT_SONAR1_TIM, TMR_C1_FLAG) != RESET){	
			if(SonarMag_1.CapStatus){														//捕获到低电平，则完成一次高电平时间获取							
				SonarMag_1.value[SonarMag_1.Index]  = STARBOT_SONAR1_TIM->c1dt;				//获取当前的捕获值.
				SonarMag_1.Index++;
				if(SonarMag_1.Index>2){
					SonarMag_1.Cal_value = 0;
					for(int i=0;i<3;i++){
						SonarMag_1.Cal_value += SonarMag_1.value[i];
					}
					SonarMag_1.Old_value = SonarMag_1.Cal_value/3.0f;
					SonarMag_1.Index = 0;
				}
				SonarMag_1.CapStatus = 0;
				STARBOT_SONAR1_TIM->ctrl1_bit.tmren = FALSE;				
				STARBOT_SONAR1_TIM->cctrl_bit.c1p        = (uint32_t)TMR_INPUT_RISING_EDGE;		//CC1P设置为上升沿捕获
				STARBOT_SONAR1_TIM->ctrl1_bit.tmren = TRUE;
			}else{																	//捕获到高电平，开始计算高电平捕获时间		
				SonarMag_1.CapStatus = 1;		
				STARBOT_SONAR1_TIM->ctrl1_bit.tmren = FALSE;	    							//关闭定时器
				STARBOT_SONAR1_TIM->cval = 0;													//清空计数器
				STARBOT_SONAR1_TIM->cctrl_bit.c1p   = (uint32_t)TMR_INPUT_FALLING_EDGE;			//CC1P设置为下降沿捕获
				STARBOT_SONAR1_TIM->ctrl1_bit.tmren = TRUE;									//使能定时器
			}	
			tmr_flag_clear(STARBOT_SONAR1_TIM, TMR_C1_FLAG);		
		}
	}		
	#endif
}
void Sonar2_IRQHandler(void)													//TIM10IRQ
{ 	
	#ifdef STM32F40_41xxx
	{	
		if(TIM_GetITStatus(STARBOT_SONAR2_TIM, TIM_IT_Update) != RESET){			
			if(SonarMag_2.CapStatus){								
				SonarMag_2.CapStatus=0;
				TIM_Cmd(STARBOT_SONAR2_TIM,DISABLE ); 	    						//关闭定时器
				TIM_SetCounter(STARBOT_SONAR2_TIM,0);								//清空计数器
				TIM_OC1PolarityConfig(STARBOT_SONAR1_TIM,TIM_ICPolarity_Rising); 	//CC1P=0 设置为上升沿捕获
				TIM_Cmd(STARBOT_SONAR2_TIM,ENABLE ); 								//使能定时器
			}
			TIM_ClearITPendingBit(STARBOT_SONAR2_TIM, TIM_IT_Update);
		}
		if(TIM_GetITStatus(STARBOT_SONAR2_TIM, TIM_IT_CC1) != RESET){	
			if(SonarMag_2.CapStatus){														//捕获到低电平，则完成一次高电平时间获取							
				SonarMag_2.value[SonarMag_2.Index]  = TIM_GetCapture1(STARBOT_SONAR2_TIM);	//获取当前的捕获值.
				SonarMag_2.Index++;
				if(SonarMag_2.Index>2){
					SonarMag_2.Cal_value = 0;
					for(int i=0;i<3;i++){
						SonarMag_2.Cal_value += SonarMag_2.value[i];
					}
					SonarMag_2.Old_value = SonarMag_2.Cal_value/3.0f;
					SonarMag_2.Index = 0;
				}
				SonarMag_2.CapStatus = 0;
				TIM_Cmd(STARBOT_SONAR2_TIM,DISABLE ); 	    						//关闭定时器
				TIM_SetCounter(STARBOT_SONAR2_TIM,0);	
				TIM_OC1PolarityConfig(STARBOT_SONAR2_TIM,TIM_ICPolarity_Rising); 	//CC1P=0 设置为上升沿捕获
				TIM_Cmd(STARBOT_SONAR2_TIM,ENABLE ); 								//使能定时器
			} else { 																//捕获到高电平，开始计算高电平捕获时间	
				SonarMag_2.CapStatus = 1;	
				TIM_Cmd(STARBOT_SONAR2_TIM,DISABLE ); 	    						//关闭定时器
				TIM_SetCounter(STARBOT_SONAR2_TIM,0);								//清空计数器
				TIM_OC1PolarityConfig(STARBOT_SONAR2_TIM,TIM_ICPolarity_Falling);	//CC1P=1 设置为下降沿捕获
				TIM_Cmd(STARBOT_SONAR2_TIM,ENABLE ); 								//使能定时器
			}
			TIM_ClearITPendingBit(STARBOT_SONAR2_TIM, TIM_IT_CC1);		
		}
	}		
	#elif AT32F40x
	{
		if(tmr_flag_get(STARBOT_SONAR2_TIM, TMR_OVF_FLAG) != RESET){	 
			if(SonarMag_2.CapStatus){								
				SonarMag_2.CapStatus=0;
				STARBOT_SONAR2_TIM->ctrl1_bit.tmren = FALSE;
				STARBOT_SONAR2_TIM->cval = 0;	
				STARBOT_SONAR2_TIM->cctrl_bit.c1p        = (uint32_t)TMR_INPUT_RISING_EDGE;		//CC1P=0 设置为上升沿捕获
				STARBOT_SONAR2_TIM->ctrl1_bit.tmren = TRUE;										//使能定时器
			}
			tmr_flag_clear(STARBOT_SONAR2_TIM, TMR_OVF_FLAG);
		}
		if(tmr_flag_get(STARBOT_SONAR2_TIM, TMR_C1_FLAG) != RESET){	
			if(SonarMag_2.CapStatus){															//捕获到低电平，则完成一次高电平时间获取							
				SonarMag_2.value[SonarMag_2.Index]  = STARBOT_SONAR2_TIM->c1dt;					//获取当前的捕获值.
				SonarMag_2.Index++;
				if(SonarMag_2.Index>2){
					SonarMag_2.Cal_value = 0;
					for(int i=0;i<3;i++){
						SonarMag_2.Cal_value += SonarMag_2.value[i];
					}
					SonarMag_2.Old_value = SonarMag_2.Cal_value/3.0f;
					SonarMag_2.Index = 0;
				}
				SonarMag_2.CapStatus = 0;
				STARBOT_SONAR2_TIM->ctrl1_bit.tmren = FALSE;	    							//关闭定时器
				STARBOT_SONAR2_TIM->cval = 0;													//清空计数器	
				STARBOT_SONAR2_TIM->cctrl_bit.c1p        = (uint32_t)TMR_INPUT_RISING_EDGE;		//CC1P设置为上升沿捕获
				STARBOT_SONAR2_TIM->ctrl1_bit.tmren = TRUE;	
			} else { 																			//捕获到高电平，开始计算高电平捕获时间	
				SonarMag_2.CapStatus = 1;	
				STARBOT_SONAR2_TIM->ctrl1_bit.tmren = FALSE;	    							//关闭定时器
				STARBOT_SONAR2_TIM->cval = 0;													//清空计数器
				STARBOT_SONAR2_TIM->cctrl_bit.c1p   = (uint32_t)TMR_INPUT_FALLING_EDGE;			//CC1P设置为下降沿捕获	
				STARBOT_SONAR2_TIM->ctrl1_bit.tmren = TRUE;										//使能定时器				
			}
			tmr_flag_clear(STARBOT_SONAR2_TIM, TMR_C1_FLAG);		
		}
	}		
	#endif	
}
void startCollectingSonar(void)
{
	switch(SonarId){
		case 0x03:
			STARBOT_SONAR1_TRIG_ON();
			STARBOT_SONAR2_TRIG_ON();
			delay_us(50);
			STARBOT_SONAR1_TRIG_OFF();
			STARBOT_SONAR2_TRIG_OFF();
			break;
		case 0x01:
			STARBOT_SONAR1_TRIG_ON();
			delay_us(50);
			STARBOT_SONAR1_TRIG_OFF();
			break;
		case 0x02:
			STARBOT_SONAR2_TRIG_ON();
			delay_us(50);
			STARBOT_SONAR2_TRIG_OFF();
			break;
	}
}

//在启动采集后50ms后读取,58.82*800 = 47056 = 47.056ms
SonarDate getSonarValue(void){	
	SonarDate gData_t;
	gData_t.Sonar1 = SonarMag_1.Old_value / 5.882; //return mm
	gData_t.Sonar2 = SonarMag_2.Old_value / 5.882; //return mm
	return gData_t;
}
void TIM1_TRG_COM_TIM11_IRQHandler(void){
	Sonar1_IRQHandler();
}
void TIM1_UP_TIM10_IRQHandler(void){
	Sonar2_IRQHandler();
}

void TMR1_TRG_HALL_TMR11_IRQHandler(void){
	Sonar1_IRQHandler();
}
void TMR1_OVF_TMR10_IRQHandler(void){
	Sonar2_IRQHandler();
}
#ifdef __cplusplus
}
#endif



