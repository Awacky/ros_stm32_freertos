#include "log_Task.h"
#include "log_handle.h"
#include "modem.h"
#include "osQueueSemap.h"
static xSemaphoreHandle xlogHandle_Mux = NULL;
uint8_t asyncBuf[2048] = {0};
uint16_t log_len = 0,write_ind = 0,read_ind = 0;
static void ReadBuf2Send(void){
	xSemaphoreTake(xlogHandle_Mux,2);//申请
	if(write_ind-read_ind>=128 || read_ind > write_ind){
		Terminal_send(&asyncBuf[read_ind],128);
		memset(&asyncBuf[read_ind],0,128);
		read_ind+=128;
		if(read_ind>=2039){
			read_ind = 0;
		}
	}
	xSemaphoreGive(xlogHandle_Mux);//释放
}
static void logHandleReadBuff(uint8_t *buf_t,uint16_t len_t){
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED){//系统已经运行		
		xSemaphoreTake(xlogHandle_Mux,2);//申请
	}
	uint16_t cValue_t = 0,cValue_t2=0;
	cValue_t = write_ind%128;
	cValue_t2 = 128 - cValue_t;
	if(cValue_t+len_t>128){
		write_ind += cValue_t2;
		memcpy(&asyncBuf[write_ind],buf_t,len_t);		
	} else {
		memcpy(&asyncBuf[write_ind],buf_t,len_t);
	}
	write_ind += len_t;
	if(write_ind>=2039){
		write_ind = 0;
	}
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED){//系统已经运行		
		xSemaphoreGive(xlogHandle_Mux);//释放
	}
}
static void elog_lock_caliback(void){
	taskENTER_CRITICAL();
}
static void elog_ulock_caliback(void){
	taskEXIT_CRITICAL();
}
static void logHandle_HTask(void *pvParameters){
	while(1){
		ReadBuf2Send();
		vTaskDelay(1);
	}
}
void logHandle_TaskInit(void){
	if(NULL == xlogHandle_Mux){
		xlogHandle_Mux = xSemaphoreCreateMutex();
		xSemaphoreGive(xlogHandle_Mux);
	}
	elog_output_register(logHandleReadBuff);
	elog_lock_register(elog_lock_caliback);
	elog_unlock_register(elog_ulock_caliback);
	elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL & ~ELOG_FMT_P_INFO);
	elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
	elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
	elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
	elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_P_INFO));
	elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_P_INFO));
	elog_start();
//	xTaskCreate(logHandle_HTask,(const char *)"logHandle_HTask",128, NULL,terminal_Pri, NULL);
}




