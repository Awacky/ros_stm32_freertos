#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _UART4LINK_TASK_H_
	#define _UART4LINK_TASK_H_
	
	#include "datatypes.h"
	
	void uart4Link_TaskInit();
	uint8_t uart4SendTxQueueTimeout(void *buff_t);
	
	
	#endif 
#ifdef __cplusplus
}
#endif


