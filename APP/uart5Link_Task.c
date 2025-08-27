
#include "uart5Link_Task.h"
#include "uart5.h"
#include "osQueueSemap.h"

#include "CustomPlot.h"
#include "device_storage.h"
#include "config_param.h"
#include "modem.h"
#include "SEGGER_RTT.h"

xQueueHandle uart5RecvQueue =NULL;
xQueueHandle uart5SendQueue =NULL;
static xSemaphoreHandle uart5CompleteBinary = NULL;
static uint8_t uart5Interfa_CallBack(const void *payload,uint16_t *payload_len);
static void uart5SendRxQueueTimeout(stcATBuff *buff_t)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uart5RecvQueue, buff_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static bool uart5ReadRxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(uart5RecvQueue,buff_t, portMAX_DELAY) == pdTRUE)
	{
		return true;
	}
	memset(buff_t->DataBuff,0,sizeof(buff_t->DataBuff));
	buff_t->length_t = 0;
	return false;
}
uint8_t uart5SendTxQueueTimeout(void *buff_t)
{
	stcATBuff *pBuff_t = (stcATBuff *)buff_t;
	if (xQueueSend(uart5SendQueue,pBuff_t,1000) == pdTRUE)
	{
		return true;
	}
	return false;
}
static bool uart5ReadTxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(uart5SendQueue,buff_t, 1000) == pdTRUE)	/*接收usbDataDelivery(1024个容量)消息*/
	{
		xSemaphoreTake(uart5CompleteBinary, 0);
		taskENTER_CRITICAL();
		u5DataFrame_Send(buff_t->DataBuff,buff_t->length_t);
		taskEXIT_CRITICAL();
			/* Wait the DMA send over & Start next */
		xSemaphoreTake(uart5CompleteBinary, 1000);
		return true;
	}
	return false;
}
static void uart5DmaSendOver_callback(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart5CompleteBinary, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void uart5Rx_HTask(void *pvParameters)
{	
	stcATBuff uart5RxBuff_t;
	uint8_t gManage_t = 0;
    for( ;; ){	
		if(uart5ReadRxQueueTimeout(&uart5RxBuff_t)){
			#ifndef Custom
			gManage_t=replyProcessing(uart5RxBuff_t.DataBuff,(uint8_t)uart5RxBuff_t.DataBuff[PACK_CMD_INDEX],(uint16_t *)&(uart5RxBuff_t.length_t),1);
			switch(gManage_t){
				case 1:{							//保存参数
					configParamGiveSemaphore();
				}break;
				case 2:{							//恢复默认参数
					restoreDefaultParam();
					configParamGiveSemaphore();
				}break;
				case 3:{							//使能UART5主动上报
					configParam.IsUsbLink = UR5_Enabled;		
				}break;
				case 4:{							//禁用UART5主动上报
					configParam.IsUsbLink = UR5_Disable;						
				}break;
				case 5:{							//CAN数据转发
					configParam.IsUsbLink = RELAID_UR5toCAN;	
				}break;
				case 6:{							//RS485(UART3)数据转发
					configParam.IsUsbLink = RELAID_UR5toUR3;	
				}break;
				case 8:{							//BLE(UART2)数据转发
					configParam.IsUsbLink = RELAID_UR5toUR2;	
				}break;
				case 9:{							//UART4数据转发
					configParam.IsUsbLink = RELAID_UR5toUR4;	
				}break;
				case 10:{							//UART1数据转发
					configParam.IsUsbLink = RELAID_UR5toUR1;	
				}break;
				case 11:{							//RS485(UART3)透明转发
					configParam.IsUsbLink = RELAID_UR5autoUR3;
				}break;
				default:{
					switch(configParam.IsUsbLink){
						case RELAID_USBtoUR5:		//USB连接上位机RS485(UART3)转发,         		把RS232(UART5)接收的数据通过USB带协议转发出去
						case RELAID_UR2toUR5:		//BLE(UART2)连接上位机RS485(UART3)转发,         把RS232(UART5)接收的数据通过BLE(UART2)带协议转发出去
						case RELAID_UR1toUR5:{		//UART1连接上位机RS485(UART3)转发,         		把RS232(UART5)接收的数据通过UART1带协议转发出去	
							memmove(&uart5RxBuff_t.DataBuff[PACK_DATA_INDEX],uart5RxBuff_t.DataBuff,uart5RxBuff_t.length_t);
							uart5RxBuff_t.DataBuff[PACK_LEN_H_INDEX] = uart5RxBuff_t.length_t>>8;
							uart5RxBuff_t.DataBuff[PACK_LEN_L_INDEX] = uart5RxBuff_t.length_t;
							uart5RxBuff_t.DataBuff[RESULTS_TYPE_INDEX] = RELAID_RS232;
							uart5RxBuff_t.length_t = sendProcessing(uart5RxBuff_t.DataBuff,CMD_PORT_RELAID);
							communicationSend_Struct(&uart5RxBuff_t,configParam.IsUsbLink);
							gManage_t = 0xFE;
						}break;
						case RELAID_UR5autoUR3:{	//RS232(UART5)连接上位机RS485(UART3)透明转发,	把RS232(UART5)接收到的数据通过RS485(UART3)直接转发出去
							communicationSend_Struct(&uart5RxBuff_t,RELAID_UR3toUR3);
						}break;
						case RELAID_UR2autoUR5:{	//BLE(UART2)连接上位机RS232(UART5)透明转发,   	把RS485(UART3)接收到的数据通过BLE(UART2)直接转发出去
							communicationSend_Struct(&uart5RxBuff_t,configParam.IsUsbLink);
						}
					}
				}break;
			}
			#endif
			if(0xFE!=gManage_t){							//不用回复
				uart5SendTxQueueTimeout(&uart5RxBuff_t);
			}
		}
	}
}
static void uart5Tx_HTask(void *pvParameters)
{	
	stcATBuff uart5TxBuff_t;
    for( ;; ) {	
		uart5ReadTxQueueTimeout(&uart5TxBuff_t);
	}
}
void uart5Link_TaskInit()
{
	if(configParam.ROS_SERIALx != SERIAL5){
		uart5RecvQueue = xQueueCreate(5, sizeof(stcATBuff));
		uart5SendQueue = xQueueCreate(5, sizeof(stcATBuff));
		uart5CompleteBinary = xSemaphoreCreateBinary();
		xSemaphoreTake(uart5CompleteBinary, 0);
		u5SetDmaRead_func(uart5SendRxQueueTimeout);
		u5SetDmaSendOver_func(uart5DmaSendOver_callback);
		#ifndef Custom
		registerComSendCallback(uart5SendTxQueueTimeout,RELAID_RS232);
		#endif
		UART5_Init(115200);
		xTaskCreate(uart5Rx_HTask,(const char *)"uart5Rx_HTask",256, NULL,uart5Rx_Pri, NULL);
		xTaskCreate(uart5Tx_HTask,(const char *)"uart5Tx_HTask",128, NULL,uart5Tx_Pri, NULL);
	}
}