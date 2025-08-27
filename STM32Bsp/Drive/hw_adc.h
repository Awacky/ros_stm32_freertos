
#ifndef _HW_ADC_H_
#define _HW_ADC_H_
	#ifdef __cplusplus
	extern "C" {
	#endif
	#include "hw_config.h"

	void Hw_ADC_Init(uint8_t Count_t,uint8_t mDriveType);
	void adcSetFCN_func(void (* const func_t)(void));
	void adcSamplingEnable(void);
	void adcSamplingDisable(void);
	float getSamplingValue(uint8_t ChName);
	float getFilterSamplingValue(uint8_t ChName);	
		
	#ifdef __cplusplus
	}
	#endif	
#endif

