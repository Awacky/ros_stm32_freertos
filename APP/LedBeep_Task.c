
#include "LedBeep_Task.h"
#include "osQueueSemap.h"
#include "hw_config.h"
#include "led.h"
static void SysLedBeep_HTask(void *pvParameters){
	if(configParam.RobotType==0){
		watchdogInit(WATCHDOG_RESET_MS);
	}
    for( ;; ){
		STARBOT_LED_RUN_Toggle();	
		vTaskDelay(300);
	}
}
void SysLedBeep_TaskInit(void){
	LED_Init();
	xTaskCreate(SysLedBeep_HTask,(const char *)"SysLedBeepHTask",128, NULL,LedBeep_Pri, NULL);
}