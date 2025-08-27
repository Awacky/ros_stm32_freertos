#ifdef __cplusplus
extern "C" {
#endif
#ifndef _DEVACTIVATE_H_
#define _DEVACTIVATE_H_

#include "datatypes.h"
void activate_init(void);
uint8_t activateSlibSet(uint8_t *Src,uint32_t len);
uint8_t getLibVerStr(uint8_t* data);
void RegistrationActErrFun(void(*ActsetError_t)(uint8_t errCode));
void getDeviceHardware(Hardware_Struct* data);
void getDeviceHardwareStr(Hardware_Struct* data);
uint16_t upGetHardware(uint8_t *upBuf_tpr,uint16_t OffsetIndex);
#endif
	
#ifdef __cplusplus
}
#endif




