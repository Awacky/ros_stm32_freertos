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
//WS2812功能函数调用
///函数名称：OperatingFun
///输入参数：InputColor->颜色
//1->红 2->绿 3->蓝 4->白 5->橙 6->黄
///输入参数：LenNum->LED灯个数
///输入参数：delay_data->延时时间
///输入参数：MainMode->模式
//1->关闭显示 2->某个颜色常亮 3->某个颜色亮灭 4->某个颜色顺时针逐个亮流水 5->某个颜色逆时针逐个亮流水
//6->某个颜色来回逐个亮流水 7->某个颜色顺时针单个亮流水 8->某个颜色逆时针单个亮流水 
//9->某个颜色来回单个亮流水10->某个颜色呼吸
	Ws2812RunMode = 4;
	for( ;; ){
		switch(Ws2812RunMode){
			case 1:{					//左转
				OperatingFun(1,6,4,100,4);
			}break;
			case 2:{					//右转
				OperatingFun(2,6,4,100,4);
			}break;
			case 3:{					//急停
				OperatingFun(3,6,4,100,3);
				vTaskDelay(300);
			}break;		
			case 4:{					//照明
				OperatingFun(3,4,4,100,2);
			}break;
			case 5:{					//原地左转
//				OperatingFun(1,6,4,100,4);
//				OperatingFun(2,6,4,100,5);
			}break;
			case 6:{					//原地右转
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