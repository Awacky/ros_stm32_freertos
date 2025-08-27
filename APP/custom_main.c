#include "main.h"
#include "config_param.h"
#include "device_storage.h"
volatile upGraderFlash flashUpStruct;
void appClearUpFalfg(void){																		//App���������־λ��ʹboot������ת��App	
	STMFLASH_Read(UP_PARAM_ADDR,(uint32_t *)&flashUpStruct,PackNum(sizeof(upGraderFlash),4));
	if(flashUpStruct.upGoCmd == UP_CMD_GOTO_APP){
		return ;
	}
	flashUpStruct.upGoCmd = UP_CMD_GOTO_APP;
	STMFLASH_Write(UP_PARAM_ADDR,(uint32_t*)(&flashUpStruct),PackNum(sizeof(upGraderFlash),4));
}
int main(void){
	#ifdef STM32F40_41xxx
		delay_init(168);								//STM32F4ʱ�ӳ�ʼ��
	#elif STM32F10X_HD
		delay_init(72);									//STM32F1ʱ�ӳ�ʼ��
	#else
		delay_init(240);								//AT32ʱ�ӳ�ʼ��
	#endif
	hw_GPIO_Init();										//Ӳ����ʼ��
	configParamInit();
	appClearUpFalfg();
//	BaseBoard_TIM6_Init();								//��ʱ��ȡ���������ݳ�ʼ��
	SysLedBeep_TaskInit();								//ϵͳLED״̬�����ʼ��
	vTaskStartScheduler();          					//�����������
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