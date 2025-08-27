#include "main.h"

//添加需要运行的任务初始化函数
PrivateFun InitTaskArr[]={
	//函数名称
	SysLedBeep_TaskInit,
//	logHandle_TaskInit,
////	#ifndef STM32F10X_HD
	MoveBase_TaskInit,
	canLink_TaskInit,
	ros_task_create,
	OledShow_TaskInit,
	Terminal_TaskInit,
	usbLink_TaskInit,
	uart1_TaskInit,
	hcBle_TaskInit,
	uart3Link_TaskInit,
	uart4Link_TaskInit,
	uart5Link_TaskInit,
	decManage_TaskInit,
	timeout_TaskInit,
	ErrorManage_TaskInit,
	Ws2812Run_TaskInit,
	#if defined ( Microros )
	ros2_task_create,
	#endif
//	#endif
	NULL//这个NULL不能删除
};
//添加需要初始化的硬件初始化函数
PrivateFun InitSetupArr[]={
	//函数名称
	hw_GPIO_Init,
	BaseBoard_TIM6_Init,
	NULL//这个NULL不能删除
};
//添加主循环要执行的函数，在OS系统中该变量不会被执行
PrivateFun InitLoopArr[]={
	//函数名称
	NULL//这个NULL不能删除
};
PrivateFun *getInitTask_Fun(void){
	return InitTaskArr;
}
PrivateFun *getInitSetup_Fun(void){
	return InitSetupArr;
}
PrivateFun *getInitLoop_Fun(void){
	return InitLoopArr;	
}
void vApplicationIdleHook( void ){
//	#ifndef STM32F10X_HD
		static u32 tickWatchdogReset = 0;
		portTickType tickCount = getSysTickCnt();
		if (tickCount - tickWatchdogReset > WATCHDOG_RESET_MS){
			tickWatchdogReset = tickCount;
			watchdogReset();
		}
//	#endif
}




