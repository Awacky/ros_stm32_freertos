#ifdef __cplusplus
extern "C" {
#endif
	#ifndef __USART1_H
	#define __USART1_H
	#include "hw_config.h"
	#define UR1ToROS
	void USART1_Init(uint32_t baudrate);
	void u1SetDmaSendOver_func(void (* const send)(void));
	void u1TaskSetDmaSendOver_func(void (* const send)(void));
	void u1SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff));
	void u1IT_IDLE_ReadEnable(void);
	void u1IT_IDLE_ReadDisable(void);
	void u1DataFrame_Send(uint8_t *buf, uint16_t length);
	void u1_printf(char* fmt,...); 
	#endif
#ifdef __cplusplus
}
#endif
