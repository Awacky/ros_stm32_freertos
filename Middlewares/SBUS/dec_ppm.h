#ifdef __cplusplus
extern "C" {
#endif

	#ifndef __DEC_PPM_H
	#define __DEC_PPM_H

	#include "hw_config.h"



	typedef struct _ppmStruct_ 
	{
		uint8_t  ppmChId;
		uint16_t ppmCapStart;
		uint16_t ppmCapEnd;
		uint16_t ppmValue[10];
	}ppmStruct;
	void dec_ppmInit(void);
	#endif
	
#ifdef __cplusplus
}
#endif
	
	
	
	
	
	
	
	
	
	
	