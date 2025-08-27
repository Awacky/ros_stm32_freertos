#ifdef __cplusplus
extern "C" {
#endif

	#ifndef __DEC_DUTY_H
	#define __DEC_DUTY_H

	#include "hw_config.h"
	
	


	typedef struct _dutyStruct_ 
	{
		uint8_t  CaptuerStatus[2];
		uint16_t dutyStart[2];
		uint16_t dutyValue[2];
	}dutyStruct;
	void dec_dutyInit(void);
	#endif
	
#ifdef __cplusplus
}
#endif