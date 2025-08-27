#ifdef __cplusplus
extern "C" {
#endif
#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H
	
#include "modem.h"

void configParamInit(void);
void restoreDefaultParam(void);
void configParamGiveSemaphore(void);

#endif /*__CONFIG_PARAM_H */
#ifdef __cplusplus
}
#endif


