#ifdef __cplusplus
extern "C" {
#endif
	#ifndef __USART3_H
	#define __USART3_H
	#include "hw_config.h"
	#define UR3ToROS
	void USART3_Init(uint32_t baudrate);
	void u3SetDmaSendOver_func(void (* const send)(void));
	void u3SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff));
	void u3IT_IDLE_ReadEnable(void);
	void u3IT_IDLE_ReadDisable(void);
	void u3DataFrame_Send(uint8_t *send_buf,uint16_t length);
	void u3_printf(char* fmt,...);	
	#endif
#ifdef __cplusplus
}
#endif


