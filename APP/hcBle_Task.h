#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _HCBLE_TASK_H_
	#define _HCBLE_TASK_H_
	
		#include "datatypes.h"
		
		void hcBle_TaskInit();
		uint8_t uart2SendTxQueueTimeout(void *buff_t);
		 
	
	#endif 
#ifdef __cplusplus
}
#endif


