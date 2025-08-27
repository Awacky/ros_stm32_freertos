#include "terminal_Task.h"
#include "osQueueSemap.h"
#include "terminal.h"
#include "CustomPlot.h"
#include "modem.h"

xQueueHandle terSendQueue =NULL;
void TerminalSend_callback(uint8_t *src_t,uint16_t srcLen){
	stcATBuff data_t;
	memset(&data_t,0,sizeof(stcATBuff));
	memcpy(data_t.DataBuff,src_t,srcLen);
	data_t.length_t = srcLen;
	xQueueSend(terSendQueue,&data_t,1000);
}	
static void terminal_HTask(void *pvParameters){	
	stcATBuff dataTask_t;
    for( ;; ) {
		if (xQueueReceive(terSendQueue,&dataTask_t,1000) == pdTRUE){	
			terminal_process_string((char *)dataTask_t.DataBuff);
		}
	}
}
void Terminal_TaskInit(void){
	#ifndef Custom
	registeTerminalSend(TerminalSend_callback);
	registeMoveVoidFun(3,CostomPlotInit);
	#endif
	terSendQueue = xQueueCreate(5, sizeof(stcATBuff));
	xTaskCreate(terminal_HTask,(const char *)"terminal_HTask",128, NULL,terminal_Pri, NULL);
}


