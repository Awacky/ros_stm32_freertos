#include "dec_duty.h"
//dutyStruct dutyValue_t;
void dec_dutyInit(void){
	#if defined(STM32F40_41xxx)
	{
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
	#else
	{
		
	}
	#endif
}
//void TIM8_BRK_TIM12_IRQHandler(void)													//TIM11IRQ
//{ 		
//	if(TIM_GetITStatus(DEC_DUTY_TIM, TIM_IT_CC1) != RESET){
//		if(dutyValue_t.CaptuerStatus[0]&0X40){														//已经捕获到高电平，当前捕获到低电平，则完成一次高电平时间获取		
//			dutyValue_t.dutyValue[0]  = TIM_GetCapture1(DEC_DUTY_TIM) - dutyValue_t.dutyStart[0];	//获取当前的捕获值.
//			dutyValue_t.CaptuerStatus[0]=0;
//			TIM_Cmd(DEC_DUTY_TIM,DISABLE ); 	    												//关闭定时器
//			TIM_OC1PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Rising); 								//CC1P=0 设置为上升沿捕获
//			TIM_Cmd(DEC_DUTY_TIM,ENABLE ); 															//使能定时器
//		} else { 																					//捕获到高电平，开始计算高电平捕获时间	
//			dutyValue_t.CaptuerStatus[0]=0;	
//			dutyValue_t.CaptuerStatus[0]|=0X40;														//标记捕获到了上升沿
//			dutyValue_t.dutyStart[0] = TIM_GetCapture1(DEC_DUTY_TIM);
//			TIM_Cmd(DEC_DUTY_TIM,DISABLE );
//			TIM_OC1PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Falling);								//CC1P=1 设置为下降沿捕获
//			TIM_Cmd(DEC_DUTY_TIM,ENABLE );
//		}
//		TIM_ClearITPendingBit(DEC_DUTY_TIM, TIM_IT_CC1);		
//	}
//	if(TIM_GetITStatus(DEC_DUTY_TIM, TIM_IT_CC2) != RESET){
//		if(dutyValue_t.CaptuerStatus[1]&0X40){														//已经捕获到高电平，当前捕获到低电平，则完成一次高电平时间获取		
//			dutyValue_t.dutyValue[1]  = TIM_GetCapture2(DEC_DUTY_TIM) - dutyValue_t.dutyStart[1];	//获取当前的捕获值.
//			dutyValue_t.CaptuerStatus[1]=0;
//			TIM_Cmd(DEC_DUTY_TIM,DISABLE );
//			TIM_OC2PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Rising); 								//CC1P=0 设置为上升沿捕获
//			TIM_Cmd(DEC_DUTY_TIM,ENABLE );
//		} else { 																					//捕获到高电平，开始计算高电平捕获时间	
//			dutyValue_t.CaptuerStatus[1]=0;	
//			dutyValue_t.CaptuerStatus[1]|=0X40;														//标记捕获到了上升沿
//			dutyValue_t.dutyStart[1] = TIM_GetCapture2(DEC_DUTY_TIM);
//			TIM_Cmd(DEC_DUTY_TIM,DISABLE );
//			TIM_OC2PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Falling);								//CC1P=1 设置为下降沿捕获
//			TIM_Cmd(DEC_DUTY_TIM,ENABLE );
//		}
//		TIM_ClearITPendingBit(DEC_DUTY_TIM, TIM_IT_CC2);		
//	}			     	    					   		
//}