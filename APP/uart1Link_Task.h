#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _UART1LINK_TASK_H_
	#define _UART1LINK_TASK_H_
	
		#include "datatypes.h"
		void uart1_TaskInit();
		uint8_t uart1SendTxQueueTimeout(void *buff_t);
	#endif 
#ifdef __cplusplus
}
#endif


