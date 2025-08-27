#ifdef __cplusplus
extern "C" {
#endif
	
#include "timeout.h"
#include "osQueueSemap.h"
// Private variables
static volatile uint32_t timeout_msec;
static volatile uint32_t last_update_time;
static volatile float timeout_brake_current;
static volatile bool has_timeout;

static void timeout_HTask(void *pvParameters){	
	for(;;) {
		if (timeout_msec != 0 && ((xTaskGetTickCount() - last_update_time) > timeout_msec)) {
			has_timeout = true;
		} else {
			has_timeout = false;
		}
		vTaskDelay(10);
	}
}
void timeout_configure(uint32_t timeout, float brake_current) {
	timeout_msec = timeout;
	timeout_brake_current = brake_current;
}
void timeout_reset(void) {
	last_update_time = xTaskGetTickCount();
}
void timeout_disable(void)
{
	timeout_msec = 0;
}
bool timeout_has_timeout(void) {
	return has_timeout;
}
uint32_t timeout_get_timeout_msec(void) {
	return timeout_msec;
}
uint32_t getElapsedTime(void){
	return (xTaskGetTickCount() - last_update_time);
}
float timeout_get_brake_current(void) {
	return timeout_brake_current;
}
void timeout_TaskInit(void){
	timeout_msec = 0;
	last_update_time = 0;
	timeout_brake_current = 0.0f;
	has_timeout = false;
	xTaskCreate(timeout_HTask,(const char *)"timeout_HTask",128, NULL,timeout_Pri, NULL);
}


#ifdef __cplusplus
}
#endif	