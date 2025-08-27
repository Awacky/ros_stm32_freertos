
#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _ERRORMANAGE_TASK_H_
	#define _ERRORMANAGE_TASK_H_
	#include "hw_config.h"
	void ErrorManage_TaskInit(void);
	void setError_Fun(uint8_t input);
	void resetError_Fun(ERROR_TypeDef input);
	#endif
#ifdef __cplusplus
}
#endif








