#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _USBLINK_TASK_H_
	#define _USBLINK_TASK_H_
	
	#include "datatypes.h"
	
	void usbLink_TaskInit();
	bool usbReadRxQueueTimeout(stcATBuff *buff_t);
	uint8_t usbSendTxQueueTimeout(void *buff_t);
	bool usbReadTxQueueTimeout(stcATBuff *buff_t);
		
	#endif 
#ifdef __cplusplus
}
#endif



