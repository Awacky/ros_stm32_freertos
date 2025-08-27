#ifndef _CANMOTOR_H_
#define _CANMOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "hw_config.h"
void canMotorInit(vMotorStr *Src_t);
void canMotorSpin(vMotorStr * M1Value,vMotorStr * M2Value);

void sdoWriteMotorSpin(vMotorStr *Src_t,uint8_t EnOsSend_t);	
void sdoReadCanMotorRpm(vMotorStr *Src_t,uint8_t EnOsSend_t);
void sdoReadCanMotorEncoder(vMotorStr *Src_t,uint8_t EnOsSend_t);
void sdoReadCanMotorBat(vMotorStr *Src_t,uint8_t EnOsSend_t);
	
void canMotorFeedbackProcessing(const canStrMsg *src);
void registeCanMotorValueFun(_Moto_Str *canDevRpm_t);
void registeCanDelayFun(void(*canDelay_t)(uint16_t daley_t));
#endif 

#ifdef __cplusplus
}
#endif



