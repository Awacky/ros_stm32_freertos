#include "main.h"
#include "config_param.h"
#include "device_storage.h"
volatile upGraderFlash flashUpStruct;
void appClearUpFalfg(void){																		//App清除升级标志位，使boot可以跳转到App	
	STMFLASH_Read(UP_PARAM_ADDR,(uint32_t *)&flashUpStruct,PackNum(sizeof(upGraderFlash),4));
	if(flashUpStruct.upGoCmd == UP_CMD_GOTO_APP){
		return ;
	}
	flashUpStruct.upGoCmd = UP_CMD_GOTO_APP;
	STMFLASH_Write(UP_PARAM_ADDR,(uint32_t*)(&flashUpStruct),PackNum(sizeof(upGraderFlash),4));
}
int main(void){
	#ifdef STM32F40_41xxx
		delay_init(168);								//STM32F4时钟初始化
	#elif STM32F10X_HD
		delay_init(72);									//STM32F1时钟初始化
	#else
		delay_init(240);								//AT32时钟初始化
	#endif
	hw_GPIO_Init();										//硬件初始化
	configParamInit();
	appClearUpFalfg();
//	BaseBoard_TIM6_Init();								//定时读取编码器数据初始化
	SysLedBeep_TaskInit();								//系统LED状态任务初始化
	vTaskStartScheduler();          					//开启任务调度
}
void vApplicationIdleHook( void ){
	#ifndef STM32F10X_HD
		static u32 tickWatchdogReset = 0;
		portTickType tickCount = getSysTickCnt();
		if (tickCount - tickWatchdogReset > WATCHDOG_RESET_MS){
			tickWatchdogReset = tickCount;
			watchdogReset();
		}
	#endif
}