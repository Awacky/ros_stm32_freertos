#ifdef __cplusplus
extern "C" {
#endif
	#ifndef __USART2_H
	#define __USART2_H	
	#include "hw_config.h"
	#define UR2ToROS
	void USART2_Init(uint32_t baudrate);
	void u2SetDmaSendOver_func(void (* const send)(void));
	void u2SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff));
	void u2IT_IDLE_ReadEnable(void);
	void u2IT_IDLE_ReadDisable(void);
	void u2DataFrame_Send(uint8_t *send_buf,uint16_t length);
	void u2_printf(char* fmt,...);
	#endif
#ifdef __cplusplus
}
#endif


