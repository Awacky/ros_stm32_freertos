
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
 			case 2:{	//ռ�ձ�ģʽ���������Ȼ��������С���н���ң�ؿ���
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
			case 4:{	//PPMģʽ ͨ��5���н���ң�ؿ���
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
			case 8:{	//PS2�ֱ����� �����ģʽ���н���
				if(!PS2_RedLight()){
					getPs2Data = PS2_Receive();
					strMV_dec.isLock = 5;
					if(getPs2Data.PS2_RX<120 || getPs2Data.PS2_RX>134){	//����X�����ҿ��Ʒ���
						strMV_dec.VelZ = (127-getPs2Data.PS2_RX)/128.0;
					}else{
						strMV_dec.VelZ = 0;
					}
					if(getPs2Data.PS2_LX<120 || getPs2Data.PS2_LX>134){ //����X������ƽ�Ʒ���
						strMV_dec.VelY = (127-getPs2Data.PS2_LX)/128.0;	
					}else{
						strMV_dec.VelY = 0;
					}
					if(getPs2Data.PS2_LY<120 || getPs2Data.PS2_LY>134){//����Y������ǰ������
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
			if(dutyValue_t.CaptuerStatus[0]&0X40){														//�Ѿ����񵽸ߵ�ƽ����ǰ���񵽵͵�ƽ�������һ�θߵ�ƽʱ���ȡ		
				dutyValue_t.dutyValue[0]  = TIM_GetCapture1(DEC_DUTY_TIM) - dutyValue_t.dutyStart[0];	//��ȡ��ǰ�Ĳ���ֵ.
				dutyValue_t.CaptuerStatus[0]=0;
				TIM_Cmd(DEC_DUTY_TIM,DISABLE ); 	    												//�رն�ʱ��
				TIM_OC1PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Rising); 								//CC1P=0 ����Ϊ�����ز���
				TIM_Cmd(DEC_DUTY_TIM,ENABLE ); 															//ʹ�ܶ�ʱ��
			} else { 																					//���񵽸ߵ�ƽ����ʼ����ߵ�ƽ����ʱ��	
				dutyValue_t.CaptuerStatus[0]=0;	
				dutyValue_t.CaptuerStatus[0]|=0X40;														//��ǲ�����������
				dutyValue_t.dutyStart[0] = TIM_GetCapture1(DEC_DUTY_TIM);
				TIM_Cmd(DEC_DUTY_TIM,DISABLE );
				TIM_OC1PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Falling);								//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(DEC_DUTY_TIM,ENABLE );
			}
			TIM_ClearITPendingBit(DEC_DUTY_TIM, TIM_IT_CC1);		
		}
		if(TIM_GetITStatus(DEC_DUTY_TIM, TIM_IT_CC2) != RESET){
			if(dutyValue_t.CaptuerStatus[1]&0X40){														//�Ѿ����񵽸ߵ�ƽ����ǰ���񵽵͵�ƽ�������һ�θߵ�ƽʱ���ȡ		
				dutyValue_t.dutyValue[1]  = TIM_GetCapture2(DEC_DUTY_TIM) - dutyValue_t.dutyStart[1];	//��ȡ��ǰ�Ĳ���ֵ.
				dutyValue_t.CaptuerStatus[1]=0;
				TIM_Cmd(DEC_DUTY_TIM,DISABLE );
				TIM_OC2PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Rising); 								//CC1P=0 ����Ϊ�����ز���
				TIM_Cmd(DEC_DUTY_TIM,ENABLE );
			} else { 																					//���񵽸ߵ�ƽ����ʼ����ߵ�ƽ����ʱ��	
				dutyValue_t.CaptuerStatus[1]=0;	
				dutyValue_t.CaptuerStatus[1]|=0X40;														//��ǲ�����������
				dutyValue_t.dutyStart[1] = TIM_GetCapture2(DEC_DUTY_TIM);
				TIM_Cmd(DEC_DUTY_TIM,DISABLE );
				TIM_OC2PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Falling);								//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(DEC_DUTY_TIM,ENABLE );
			}
			TIM_ClearITPendingBit(DEC_DUTY_TIM, TIM_IT_CC2);		
		}
	}
	#else
	{
		if(tmr_flag_get(DEC_DUTY_TIM, TMR_C1_RECAPTURE_FLAG) != RESET){	 
			dutyValue_t.dutyValue[0]  = tmr_div_value_get(DEC_DUTY_TIM) - dutyValue_t.dutyStart[0];	//��ȡ��ǰ�Ĳ���ֵ.
			dutyValue_t.CaptuerStatus[0]=0;
			DEC_DUTY_TIM->ctrl1_bit.tmren = 0;														//�رն�ʱ��	    												
//			TIM_OC1PolarityConfig(DEC_DUTY_TIM,TIM_ICPolarity_Rising); 								//CC1P=0 ����Ϊ�����ز���
			DEC_DUTY_TIM->ctrl1_bit.tmren = 1;														//������ʱ��	 	
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
			TIM_Cmd(DEC_PPM_TIM,DISABLE ); 								//�رն�ʱ��
			TIM_SetCounter(DEC_PPM_TIM,0);
			TIM_Cmd(DEC_PPM_TIM,ENABLE );								//ʹ�ܶ�ʱ��
			TIM_ClearITPendingBit(DEC_PPM_TIM,TIM_IT_Update);
		}
		if(TIM_GetITStatus(DEC_PPM_TIM, TIM_IT_CC2) != RESET){
			if(ppmValue_t.ppmChId ==0){
				TIM_Cmd(DEC_PPM_TIM,DISABLE ); 								//�رն�ʱ�� 	   
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
					TIM_Cmd(DEC_PPM_TIM,DISABLE ); 								//�رն�ʱ�� 	   
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





