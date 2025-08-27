#ifdef __cplusplus
extern "C" {
#endif
	#ifndef __UART5_H
	#define __UART5_H
	#include "hw_config.h"
	#define UR5ToROS
	void UART5_Init(uint32_t baudrate);
	void u5SetDmaSendOver_func(void (* const send)(void));
	void u5SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff));
	void u5IT_IDLE_ReadEnable(void);
	void u5IT_IDLE_ReadDisable(void);
	void u5DataFrame_Send(uint8_t *send_buf,uint16_t length);
	void u5_printf(char* fmt,...);
	#endif
#ifdef __cplusplus
}
#endif


