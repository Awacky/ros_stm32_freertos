#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _UART5LINK_TASK_H_
	#define _UART5LINK_TASK_H_
	
		#include "datatypes.h"
		
		void uart5Link_TaskInit();
		uint8_t uart5SendTxQueueTimeout(void *buff_t);
		 
	#endif 
#ifdef __cplusplus
}
#endif


