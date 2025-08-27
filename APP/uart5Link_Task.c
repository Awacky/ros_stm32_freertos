
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
	if (xQueueReceive(uart5SendQueue,buff_t, 1000) == pdTRUE)	/*����usbDataDelivery(1024������)��Ϣ*/
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
				case 1:{							//�������
					configParamGiveSemaphore();
				}break;
				case 2:{							//�ָ�Ĭ�ϲ���
					restoreDefaultParam();
					configParamGiveSemaphore();
				}break;
				case 3:{							//ʹ��UART5�����ϱ�
					configParam.IsUsbLink = UR5_Enabled;		
				}break;
				case 4:{							//����UART5�����ϱ�
					configParam.IsUsbLink = UR5_Disable;						
				}break;
				case 5:{							//CAN����ת��
					configParam.IsUsbLink = RELAID_UR5toCAN;	
				}break;
				case 6:{							//RS485(UART3)����ת��
					configParam.IsUsbLink = RELAID_UR5toUR3;	
				}break;
				case 8:{							//BLE(UART2)����ת��
					configParam.IsUsbLink = RELAID_UR5toUR2;	
				}break;
				case 9:{							//UART4����ת��
					configParam.IsUsbLink = RELAID_UR5toUR4;	
				}break;
				case 10:{							//UART1����ת��
					configParam.IsUsbLink = RELAID_UR5toUR1;	
				}break;
				case 11:{							//RS485(UART3)͸��ת��
					configParam.IsUsbLink = RELAID_UR5autoUR3;
				}break;
				default:{
					switch(configParam.IsUsbLink){
						case RELAID_USBtoUR5:		//USB������λ��RS485(UART3)ת��,         		��RS232(UART5)���յ�����ͨ��USB��Э��ת����ȥ
						case RELAID_UR2toUR5:		//BLE(UART2)������λ��RS485(UART3)ת��,         ��RS232(UART5)���յ�����ͨ��BLE(UART2)��Э��ת����ȥ
						case RELAID_UR1toUR5:{		//UART1������λ��RS485(UART3)ת��,         		��RS232(UART5)���յ�����ͨ��UART1��Э��ת����ȥ	
							memmove(&uart5RxBuff_t.DataBuff[PACK_DATA_INDEX],uart5RxBuff_t.DataBuff,uart5RxBuff_t.length_t);
							uart5RxBuff_t.DataBuff[PACK_LEN_H_INDEX] = uart5RxBuff_t.length_t>>8;
							uart5RxBuff_t.DataBuff[PACK_LEN_L_INDEX] = uart5RxBuff_t.length_t;
							uart5RxBuff_t.DataBuff[RESULTS_TYPE_INDEX] = RELAID_RS232;
							uart5RxBuff_t.length_t = sendProcessing(uart5RxBuff_t.DataBuff,CMD_PORT_RELAID);
							communicationSend_Struct(&uart5RxBuff_t,configParam.IsUsbLink);
							gManage_t = 0xFE;
						}break;
						case RELAID_UR5autoUR3:{	//RS232(UART5)������λ��RS485(UART3)͸��ת��,	��RS232(UART5)���յ�������ͨ��RS485(UART3)ֱ��ת����ȥ
							communicationSend_Struct(&uart5RxBuff_t,RELAID_UR3toUR3);
						}break;
						case RELAID_UR2autoUR5:{	//BLE(UART2)������λ��RS232(UART5)͸��ת��,   	��RS485(UART3)���յ�������ͨ��BLE(UART2)ֱ��ת����ȥ
							communicationSend_Struct(&uart5RxBuff_t,configParam.IsUsbLink);
						}
					}
				}break;
			}
			#endif
			if(0xFE!=gManage_t){							//���ûظ�
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