
#include "decManage_Task.h"
#include "osQueueSemap.h"
#include "moveBase_Task.h"
#include "uart5.h"
#include "pstwo.h"
static uint8_t InputDevNum = 0; // 8:PS2,4:ppm,2:duty
static PSTwo_Data getPs2Data;	
//static ppmStruct ppmValue_t;
//static dutyStruct dutyValue_t;

ppmStruct ppmValue_t;
dutyStruct dutyValue_t;

static stcMoveVel strMV_dec;
static uint16_t velRatioZ = 0;
void dec_InputDevInit(uint8_t Name);
static void decManage_HTask(void *pvParameters){
    for( ;; ){	
		switch(InputDevNum){
 			case 2:{	//占空比模式先拉到最大，然后拉到最小进行解锁遥控控制
				if(dutyValue_t.dutyValue[0]>1650 && dutyValue_t.dutyValue[1]>1900){
					vTaskDelay(500);
					if(dutyValue_t.dutyValue[0]>1650 && dutyValue_t.dutyValue[1]>1900){
						strMV_dec.isLock = 1;
					}
				} else if(dutyValue_t.dutyValue[0]<1380 && dutyValue_t.dutyValue[1]<1100){
					vTaskDelay(500);
					if(dutyValue_t.dutyValue[0]<1380 && dutyValue_t.dutyValue[1]<1100){
						if(1 == strMV_dec.isLock){
							strMV_dec.isLock = 0;
							setMoveVelData(&strMV_dec);
							strMV_dec.isLock = 5;
						}
					}
				} else if(5 == strMV_dec.isLock){
					strMV_dec.VelX = (1500 - dutyValue_t.dutyValue[1])/450.f;
					strMV_dec.VelZ = (1500 - dutyValue_t.dutyValue[0])/350.f;
					if(fabs(strMV_dec.VelX)<0.1)strMV_dec.VelX = 0;
					if(fabs(strMV_dec.VelZ)<0.1)strMV_dec.VelZ = 0;
					if(fabs(strMV_dec.VelY)<0.1)strMV_dec.VelY = 0;
					setMoveVelData(&strMV_dec);
				}
				vTaskDelay(50);
			}break;
			case 4:{	//PPM模式 通道5进行解锁遥控控制
				if(ppmValue_t.ppmValue[5]>1500){
					strMV_dec.isLock = 5;
					velRatioZ = (ppmValue_t.ppmValue[6] - 960) / 100;
					strMV_dec.VelX = (1500 - ppmValue_t.ppmValue[1])/450.f;				//X
					strMV_dec.VelZ = (1500 - ppmValue_t.ppmValue[0])/450.f*velRatioZ;	//Z
					strMV_dec.VelY = (1500 - ppmValue_t.ppmValue[3])/450.f;				//Y
					if(fabs(strMV_dec.VelX)<0.1)strMV_dec.VelX = 0;
					if(fabs(strMV_dec.VelZ)<0.1)strMV_dec.VelZ = 0;
					if(fabs(strMV_dec.VelY)<0.1)strMV_dec.VelY = 0;
					setMoveVelData(&strMV_dec);
				}else{
					if(5 == strMV_dec.isLock){
						strMV_dec.isLock = 0;
						setMoveVelData(&strMV_dec);
						strMV_dec.isLock = 2;
					}
				}
				vTaskDelay(50);
			}break;
			case 8:{	//PS2手柄控制 按红灯模式进行解锁
				if(!PS2_RedLight()){
					getPs2Data = PS2_Receive();
					strMV_dec.isLock = 5;
					if(getPs2Data.PS2_RX<120 || getPs2Data.PS2_RX>134){	//右手X轴左右控制方向
						strMV_dec.VelZ = (127-getPs2Data.PS2_RX)/128.0;
					}else{
						strMV_dec.VelZ = 0;
					}
					if(getPs2Data.PS2_LX<120 || getPs2Data.PS2_LX>134){ //左手X轴左右平移方向
						strMV_dec.VelY = (127-getPs2Data.PS2_LX)/128.0;	
					}else{
						strMV_dec.VelY = 0;
					}
					if(getPs2Data.PS2_LY<120 || getPs2Data.PS2_LY>134){//左手Y轴左右前进方向
					   strMV_dec.VelX = (127-getPs2Data.PS2_LY )/128.0;
					}else{
						strMV_dec.VelX = 0;
					}
					setMoveVelData(&strMV_dec);
				} else {
					if(5 == strMV_dec.isLock){
						strMV_dec.isLock = 0;
						setMoveVelData(&strMV_dec);
						strMV_dec.isLock = 2;
					}
				}
				vTaskDelay(100);
			}break;
			default:{
				vTaskDelay(500);
			}break;
		}		
	}
}
void decManage_TaskInit(void){
	InputDevNum = (configParam.ControlMode & 0xFE);
	dec_InputDevInit(InputDevNum);
	xTaskCreate(decManage_HTask,(const char *)"decManage_HTask",256, NULL,decManage_Pri, NULL);
}
void dec_InputDevInit(uint8_t Name){
	InputDevNum = Name;
	memset(&dutyValue_t,0,sizeof(dutyStruct));
	memset(&ppmValue_t,0,sizeof(ppmStruct));
	memset(&getPs2Data,0,sizeof(PSTwo_Data));
	memset(&strMV_dec,0,sizeof(stcMoveVel));
	switch(InputDevNum){
		case 2:{
			dec_dutyInit();
		}break;
		case 4:{
			dec_ppmInit();
		}break;
		case 8:{
			PS2_SetInit();
		}break;
	}
}
void dec_dutyManage(void){
	#if defined(STM32F40_41xxx)
	{
		if(TIM_GetITStatus(DEC_DUTY_TIM, TIM_IT_CC1) != RESET){
			if(dutyValue_t.CaptuerStatus[0]&0X40){														//已经捕获到高电平，当前捕获到低电平，则完成一次高电平时间获取		
				dutyValue_t.dutyValue[0]  = TIM_GetCapture1(DEC_DUTY_TIM) - dutyValue_t.dutyStart[0];	//获取当前的捕获值.
				dutyValue_t.CaptuerStatus[0]=0;
				TIM_Cmd(DEC_DUTY_TIM,DISABLE ); 	    												//关闭定时器
				TIM_OC1PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Rising); 								//CC1P=0 设置为上升沿捕获
				TIM_Cmd(DEC_DUTY_TIM,ENABLE ); 															//使能定时器
			} else { 																					//捕获到高电平，开始计算高电平捕获时间	
				dutyValue_t.CaptuerStatus[0]=0;	
				dutyValue_t.CaptuerStatus[0]|=0X40;														//标记捕获到了上升沿
				dutyValue_t.dutyStart[0] = TIM_GetCapture1(DEC_DUTY_TIM);
				TIM_Cmd(DEC_DUTY_TIM,DISABLE );
				TIM_OC1PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Falling);								//CC1P=1 设置为下降沿捕获
				TIM_Cmd(DEC_DUTY_TIM,ENABLE );
			}
			TIM_ClearITPendingBit(DEC_DUTY_TIM, TIM_IT_CC1);		
		}
		if(TIM_GetITStatus(DEC_DUTY_TIM, TIM_IT_CC2) != RESET){
			if(dutyValue_t.CaptuerStatus[1]&0X40){														//已经捕获到高电平，当前捕获到低电平，则完成一次高电平时间获取		
				dutyValue_t.dutyValue[1]  = TIM_GetCapture2(DEC_DUTY_TIM) - dutyValue_t.dutyStart[1];	//获取当前的捕获值.
				dutyValue_t.CaptuerStatus[1]=0;
				TIM_Cmd(DEC_DUTY_TIM,DISABLE );
				TIM_OC2PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Rising); 								//CC1P=0 设置为上升沿捕获
				TIM_Cmd(DEC_DUTY_TIM,ENABLE );
			} else { 																					//捕获到高电平，开始计算高电平捕获时间	
				dutyValue_t.CaptuerStatus[1]=0;	
				dutyValue_t.CaptuerStatus[1]|=0X40;														//标记捕获到了上升沿
				dutyValue_t.dutyStart[1] = TIM_GetCapture2(DEC_DUTY_TIM);
				TIM_Cmd(DEC_DUTY_TIM,DISABLE );
				TIM_OC2PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Falling);								//CC1P=1 设置为下降沿捕获
				TIM_Cmd(DEC_DUTY_TIM,ENABLE );
			}
			TIM_ClearITPendingBit(DEC_DUTY_TIM, TIM_IT_CC2);		
		}
	}
	#else
	{
		if(tmr_flag_get(DEC_DUTY_TIM, TMR_C1_RECAPTURE_FLAG) != RESET){	 
			dutyValue_t.dutyValue[0]  = tmr_div_value_get(DEC_DUTY_TIM) - dutyValue_t.dutyStart[0];	//获取当前的捕获值.
			dutyValue_t.CaptuerStatus[0]=0;
			DEC_DUTY_TIM->ctrl1_bit.tmren = 0;														//关闭定时器	    												
//			TIM_OC1PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Rising); 								//CC1P=0 设置为上升沿捕获
			DEC_DUTY_TIM->ctrl1_bit.tmren = 1;														//开启定时器	 	
			tmr_flag_clear(DEC_DUTY_TIM, TMR_C1_RECAPTURE_FLAG); 			
		}				   
		  
	}
	#endif	
}
void dec_ppmManage(void){
	#if defined(STM32F40_41xxx)
	{
		if(TIM_GetITStatus(DEC_PPM_TIM, TIM_IT_Update) != RESET){	
			ppmValue_t.ppmChId = 0;		
			TIM_Cmd(DEC_PPM_TIM,DISABLE ); 								//关闭定时器
			TIM_SetCounter(DEC_PPM_TIM,0);
			TIM_Cmd(DEC_PPM_TIM,ENABLE );								//使能定时器
			TIM_ClearITPendingBit(DEC_PPM_TIM,TIM_IT_Update);
		}
		if(TIM_GetITStatus(DEC_PPM_TIM, TIM_IT_CC2) != RESET){
			if(ppmValue_t.ppmChId ==0){
				TIM_Cmd(DEC_PPM_TIM,DISABLE ); 								//关闭定时器 	   
				TIM_SetCounter(DEC_PPM_TIM,0);				
				TIM_Cmd(DEC_PPM_TIM,ENABLE );
				ppmValue_t.ppmCapStart = 0;
				ppmValue_t.ppmChId++;
				TIM_ClearITPendingBit(DEC_PPM_TIM, TIM_IT_CC2);	
				return;
			}
			if(ppmValue_t.ppmChId){
				ppmValue_t.ppmCapEnd = TIM_GetCapture2(DEC_PPM_TIM);
				ppmValue_t.ppmValue[ppmValue_t.ppmChId-1] = ppmValue_t.ppmCapEnd - ppmValue_t.ppmCapStart;
				ppmValue_t.ppmCapStart = ppmValue_t.ppmCapEnd;
				if(ppmValue_t.ppmValue[ppmValue_t.ppmChId-1]>2100){
					TIM_Cmd(DEC_PPM_TIM,DISABLE ); 								//关闭定时器 	   
					TIM_SetCounter(DEC_PPM_TIM,0);				
					TIM_Cmd(DEC_PPM_TIM,ENABLE );
					ppmValue_t.ppmCapStart = 0;
					ppmValue_t.ppmChId =0;
					ppmValue_t.ppmChId++;
					TIM_ClearITPendingBit(DEC_PPM_TIM, TIM_IT_CC2);	
					return;
				}
				ppmValue_t.ppmChId++;
				if(ppmValue_t.ppmChId>10){
					ppmValue_t.ppmChId=0; 
				}
			}
			TIM_ClearITPendingBit(DEC_PPM_TIM, TIM_IT_CC2);			
		}	
	}
	# else
	{
		
	}
	#endif
}

void TIM8_BRK_TIM12_IRQHandler(void){
	switch(InputDevNum){
		case 2:{
			dec_dutyManage();
		}break;
		case 4:{
			dec_ppmManage();
		}break;
	}
}

void TMR8_BRK_TMR12_IRQHandler(void){
	switch(InputDevNum){
		case 2:{
			dec_dutyManage();
		}break;
		case 4:{
			dec_ppmManage();
		}break;
	}
}





