#ifdef __cplusplus
extern "C" {
#endif
	#ifndef __UART4_H
	#define __UART4_H
	#include "hw_config.h"
	#define UR4ToROS
	void UART4_Init(uint32_t baudrate);
	void u4SetDmaSendOver_func(void (* const send)(void));
	void u4SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff));
	void u4IT_IDLE_ReadEnable(void);
	void u4IT_IDLE_ReadDisable(void);
	void u4DataFrame_Send(uint8_t *send_buf,uint16_t length);
	void u4_printf(char* fmt,...);
	#endif
#ifdef __cplusplus
}
#endif


