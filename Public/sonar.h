#ifdef __cplusplus
extern "C" {
#endif

	#ifndef __SONAR_H
	#define __SONAR_H

	#include "hw_config.h"
	
	void Sonar_1_TRIG_Enabled(void);
	void Sonar_2_TRIG_Enabled(void);
	void Sonar_Init(uint8_t SonarID);
	SonarDate getSonarValue(void);
	void startCollectingSonar(void);
	#endif
	
#ifdef __cplusplus
}
#endif



