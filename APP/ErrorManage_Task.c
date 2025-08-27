 
#include "ErrorManage_Task.h"
#include "osQueueSemap.h"
#include "hw_config.h"
#include "devActivate.h"
#include "led.h"
ERROR_TypeDef err_Coder = NONE_Er;
uint8_t errorPos = 0;
uint8_t Beep_st = 1;
void setError_Fun(uint8_t input){
	err_Coder |= input;
}
void resetError_Fun(ERROR_TypeDef input){
	err_Coder &= (~input);
}
static void ErrorManage_HTask(void *pvParameters)
{	
	for( ;; ){
		switch(Beep_st){
			case 1:{
				STARBOT_BEEP_On();
				vTaskDelay(300);
				STARBOT_BEEP_Off();
				vTaskDelay(50);
				Beep_st = 0;
			}break;
			case 2:{
				STARBOT_BEEP_On();
				vTaskDelay(300);
				STARBOT_BEEP_Off();
				vTaskDelay(50);
				STARBOT_BEEP_On();
				vTaskDelay(50);
				STARBOT_BEEP_Off();
				vTaskDelay(50);
				STARBOT_BEEP_On();
				vTaskDelay(100);
				STARBOT_BEEP_Off();
				vTaskDelay(50);
				Beep_st = 255;
			}break;
		}
		if((err_Coder)&(1<<errorPos)){
			for(uint8_t j=0;j<((errorPos+3)*2);j++){
				STARBOT_LED_FAULT_Toggle();
				STARBOT_BEEP_Toggle();
				vTaskDelay(300);
			}
			switch(errorPos){
				case 4:										//参数保存失败报警操作失败一次报一次
				case 5:{									//急停报警
					err_Coder = err_Coder &(~(1<<errorPos));
				}break;
				default:break;
			}
		}
		errorPos++;
		if(errorPos>7){
			errorPos=0;
			if(0==err_Coder && 0 == Beep_st){
				Beep_st=2;
			}
		}
		STARBOT_LED_FAULT_Off();
		vTaskDelay(700);
	}
}
void ErrorManage_TaskInit(void)
{
	#ifndef Custom
	RegistrationActErrFun(setError_Fun);
	#endif
	xTaskCreate(ErrorManage_HTask,(const char *)"ErrorManageHTask",128, NULL,ErrorManage_Pri, NULL);
}