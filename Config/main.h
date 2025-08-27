#ifndef _MAIN_H_
#define _MAIN_H_

#include "delay.h"
#include "osQueueSemap.h"
#include "ros_bsp.h"
#include "ros_node.h"
#include "LedBeep_Task.h"
#include "config_param.h"
#include "OledShow_Task.h"
#include "moveBase_Task.h"
#include "usbLink_Task.h"
#include "uart1Link_Task.h"
#include "hcBle_Task.h"
#include "uart3Link_Task.h"
#include "uart4Link_Task.h"
#include "uart5Link_Task.h"
#include "decManage_Task.h"
#include "canLink_Task.h"
#include "ErrorManage_Task.h"
#include "ws2812Run_task.h"
#include "terminal_Task.h"
#if defined ( Microros )
	#include "microros_tasks.h"
#endif
#include "timeout.h"

typedef void(* PrivateFun)(void);
PrivateFun *getInitTask_Fun(void);
PrivateFun *getInitSetup_Fun(void);
PrivateFun *getInitLoop_Fun(void);
#endif 




