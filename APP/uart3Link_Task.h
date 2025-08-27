#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _UART3LINK_TASK_H_
	#define _UART3LINK_TASK_H_
	
		#include "datatypes.h"
		
		void uart3Link_TaskInit();
		uint8_t uart3SendTxQueueTimeout(void *buff_t);
	
	#endif 
#ifdef __cplusplus
}
#endif



