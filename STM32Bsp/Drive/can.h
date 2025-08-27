#ifdef __cplusplus
extern "C" {
#endif
	
	#ifndef __CAN_H
	#define __CAN_H	
	#include "hw_config.h"    							    
		
	void HwCan_Init(void);	
	uint8_t canComm_Transmit(void *Src);
	void canComm_Transmit_Sid(uint32_t id, uint8_t *data, uint8_t len);
	void canComm_Transmit_Eid(uint32_t id, uint8_t *data, uint8_t len);
	void canSetosRead_func(void(*osRead_func_t)(canStrMsg *getStcABuff));	

	#endif

#ifdef __cplusplus
}
#endif




























