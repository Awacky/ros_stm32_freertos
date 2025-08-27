#include "ws2812Run_task.h"
#include "WS2812.h"
#include "osQueueSemap.h"
static uint8_t EnWs2812Run = 0;
static uint8_t Ws2812RunMode = 0;

void setRunMode_Fun(uint8_t input){
	Ws2812RunMode = input;
}
static void Ws2812Run_HTask(void *pvParameters)
{	
//WS2812���ܺ�������
///�������ƣ�OperatingFun
///���������InputColor->��ɫ
//1->�� 2->�� 3->�� 4->�� 5->�� 6->��
///���������LenNum->LED�Ƹ���
///���������delay_data->��ʱʱ��
///���������MainMode->ģʽ
//1->�ر���ʾ 2->ĳ����ɫ���� 3->ĳ����ɫ���� 4->ĳ����ɫ˳ʱ���������ˮ 5->ĳ����ɫ��ʱ���������ˮ
//6->ĳ����ɫ�����������ˮ 7->ĳ����ɫ˳ʱ�뵥������ˮ 8->ĳ����ɫ��ʱ�뵥������ˮ 
//9->ĳ����ɫ���ص�������ˮ10->ĳ����ɫ����
	Ws2812RunMode = 4;
	for( ;; ){
		switch(Ws2812RunMode){
			case 1:{					//��ת
				OperatingFun(1,6,4,100,4);
			}break;
			case 2:{					//��ת
				OperatingFun(2,6,4,100,4);
			}break;
			case 3:{					//��ͣ
				OperatingFun(3,6,4,100,3);
				vTaskDelay(300);
			}break;		
			case 4:{					//����
				OperatingFun(3,4,4,100,2);
			}break;
			case 5:{					//ԭ����ת
//				OperatingFun(1,6,4,100,4);
//				OperatingFun(2,6,4,100,5);
			}break;
			case 6:{					//ԭ����ת
//				OperatingFun(1,6,4,100,5);
//				OperatingFun(2,6,4,100,4);
			}break;
			default:{
				OperatingFun(3,6,4,100,1);
			}break;
		}
		vTaskDelay(200);
	}
}
static void Ws2812Delay(uint16_t delay_t){
	vTaskDelay(delay_t);
}

void Ws2812Run_TaskInit(void)
{	
	EnWs2812Run = configParam.perEnabled.rgbled;
	if(EnWs2812Run){
		if(configParam.RobotType == ROBOT_D2 || configParam.RobotType == ROBOT_A1 || 
		   configParam.RobotType == ROBOT_A2 || configParam.RobotType == ROBOT_O3){
			RegistrationWs2812Fun(Ws2812Delay,Ws2812TimeInit,Ws2812SendBuffDma);
			EnWs2812Run = 3;
			WS2812_Init(EnWs2812Run);
			xTaskCreate(Ws2812Run_HTask,(const char *)"Ws2812Run_HTask",128, NULL,Ws2812Run_Pri, NULL);
		}
	}
}