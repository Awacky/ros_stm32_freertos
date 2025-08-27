#ifdef __cplusplus
extern "C" {
#endif

	#ifndef TIMEOUT_H_
	#define TIMEOUT_H_
	#include "hw_config.h"
	// Functions
	void timeout_TaskInit(void);
	void timeout_configure(uint32_t timeout, float brake_current);
	void timeout_reset(void);
	void timeout_disable(void);
	bool timeout_has_timeout(void);
	uint32_t timeout_get_timeout_msec(void);
	float timeout_get_brake_current(void);
	uint32_t getElapsedTime(void);
	#endif /* TIMEOUT_H_ */
	
#ifdef __cplusplus
}
#endif	



