#ifdef __cplusplus
extern "C" {
#endif


#ifndef _WS2812_H
#define _WS2812_H
#include "string.h "
#include "stdint.h"
void RegistrationWs2812Fun(void(*wsDelay_func_t)(uint16_t daley_t),void(*wsLedInit_func_t)(uint8_t LedType),\
					   void(*wsLedSend_func_t)(uint8_t LedType,uint16_t *sendBuff,uint16_t buffLen));
void WS2812_Init(uint8_t ledType);
void OperatingFun(uint8_t ledType,uint16_t InputColor,uint16_t LenNum,uint16_t delay_data,uint16_t MainMode);

#endif


#ifdef __cplusplus
}
#endif

