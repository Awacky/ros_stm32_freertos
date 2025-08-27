#include "dec_ppm.h"
//ppmStruct srcValue_t;
void dec_ppmInit(void){
	#if defined(STM32F40_41xxx)
	{
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
	#else
	{
		
	}
	#endif
}

//void TIM8_BRK_TIM12_IRQHandler(void)													//TIM11IRQ
//{ 		
//	if(TIM_GetITStatus(DEC_PPM_TIM, TIM_IT_Update) != RESET){	
//		srcValue_t.ppmChId = 0;		
//		TIM_Cmd(DEC_PPM_TIM,DISABLE ); 								//关闭定时器
//		TIM_SetCounter(DEC_PPM_TIM,0);
//		TIM_Cmd(DEC_PPM_TIM,ENABLE );								//使能定时器
//		TIM_ClearITPendingBit(DEC_PPM_TIM,TIM_IT_Update);
//	}
//	if(TIM_GetITStatus(DEC_PPM_TIM, TIM_IT_CC2) != RESET){
//		if(srcValue_t.ppmChId ==0){
//			TIM_Cmd(DEC_PPM_TIM,DISABLE ); 								//关闭定时器 	   
//			TIM_SetCounter(DEC_PPM_TIM,0);				
//			TIM_Cmd(DEC_PPM_TIM,ENABLE );
//			srcValue_t.ppmCapStart = 0;
//			srcValue_t.ppmChId++;
//			TIM_ClearITPendingBit(DEC_PPM_TIM, TIM_IT_CC2);	
//			return;
//		}
//		if(srcValue_t.ppmChId){
//			srcValue_t.ppmCapEnd = TIM_GetCapture2(DEC_PPM_TIM);
//			srcValue_t.ppmValue[srcValue_t.ppmChId-1] = srcValue_t.ppmCapEnd - srcValue_t.ppmCapStart;
//			srcValue_t.ppmCapStart = srcValue_t.ppmCapEnd;
//			if(srcValue_t.ppmValue[srcValue_t.ppmChId-1]>2100){
//				TIM_Cmd(DEC_PPM_TIM,DISABLE ); 								//关闭定时器 	   
//				TIM_SetCounter(DEC_PPM_TIM,0);				
//				TIM_Cmd(DEC_PPM_TIM,ENABLE );
//				srcValue_t.ppmCapStart = 0;
//				srcValue_t.ppmChId =0;
//				srcValue_t.ppmChId++;
//				return;
//			}
//			srcValue_t.ppmChId++;
//			if(srcValue_t.ppmChId>10){
//				srcValue_t.ppmChId=9; 
//			}
//		}
//		TIM_ClearITPendingBit(DEC_PPM_TIM, TIM_IT_CC2);			
//	}			     	    					   		
//}






