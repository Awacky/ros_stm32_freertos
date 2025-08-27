
#include "uart1Link_Task.h"
#include "usart1.h"
#include "osQueueSemap.h"
#include "config_param.h"
#include "SEGGER_RTT.h"
#include "CustomPlot.h"
#include "ros_bsp.h"
xQueueHandle uart1RecvQueue =NULL;
xQueueHandle uart1SendQueue =NULL;
static uint8_t u1ToWinTools = 0;
static xSemaphoreHandle uart1CompleteBinary = NULL;
static uint8_t uart1Interfa_CallBack(const void *payload,uint16_t *payload_len);
static void uart1SendRxQueueTimeout(stcATBuff *buff_t)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uart1RecvQueue, buff_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static void uart1ToToolsDisable(void){
	u1ToWinTools = 0;
}
static void uart1ToToolsSendRxQueueTimeout(stcATBuff *buff_t)
{
	u1ToWinTools = 1;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uart1RecvQueue, buff_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static bool uart1ReadRxQueueTimeout(stcATBuff *buff_t)
{
	if(xQueueReceive(uart1RecvQueue,buff_t, 5) == pdTRUE)
	{
		return true;
	}
	memset(buff_t->DataBuff,0,sizeof(buff_t->DataBuff));
	buff_t->length_t = 0;
	return false;
}
uint8_t uart1SendTxQueueTimeout(void *buff_t)
{
	stcATBuff *pBuff_t = (stcATBuff *)buff_t;
	if(u1ToWinTools){
		#ifndef Custom
		ros_serial_write_length(pBuff_t->DataBuff,pBuff_t->length_t);
		#endif
		return true;
		
	}
	if (xQueueSend(uart1SendQueue,pBuff_t,1000) == pdTRUE){
		return true;
	}
	return false;
}
static bool uart1ReadTxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(uart1SendQueue,buff_t,10) == pdTRUE)
	{	
		xSemaphoreTake(uart1CompleteBinary, 0);
		taskENTER_CRITICAL();
		u1DataFrame_Send(buff_t->DataBuff,buff_t->length_t);
		taskEXIT_CRITICAL();
			/* Wait the DMA send over & Start next */
		xSemaphoreTake(uart1CompleteBinary, 1000);
		return true;
	}
	return false;
}
static void uart1DmaSendOver_callback(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart1CompleteBinary, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
static void uart1Rx_HTask(void *pvParameters)
{	
	stcATBuff uart1Buff_t;
	uint8_t gManage_t = 0;
	while(1){
		if(uart1ReadRxQueueTimeout(&uart1Buff_t)){
			#ifndef Custom
			gManage_t=replyProcessing(uart1Buff_t.DataBuff,(uint8_t)uart1Buff_t.DataBuff[PACK_CMD_INDEX],(uint16_t *)&(uart1Buff_t.length_t),1);
			switch(gManage_t){
				case 1:{							//�������
					configParamGiveSemaphore();
				}break;
				case 2:{							//�ָ�Ĭ�ϲ���
					restoreDefaultParam();
					configParamGiveSemaphore();
				}break;
				case 3:{							//ʹ��UART1�����ϱ�
					configParam.IsUsbLink = UR1_Enabled;		
				}break;
				case 4:{							//����UART1�����ϱ�
					configParam.IsUsbLink = UR1_Disable;		
				}break;
				case 5:{							//����1������λ��CAN����ת��
					configParam.IsUsbLink = RELAID_UR1toCAN;	
				}break;
				case 6:{							//����1������λ��RS485(UART3)����ת��
					configParam.IsUsbLink = RELAID_UR1toUR3;	
				}break;
				case 7:{							//����1������λ��RS232(UART5)����ת��
					configParam.IsUsbLink = RELAID_UR1toUR5;	
				}break;
				case 8:{							//����1������λ��BLE(UART2)����ת��
					configParam.IsUsbLink = RELAID_UR1toUR2;	
				}break;
				case 9:{							//����1������λ��UART4����ת��
					configParam.IsUsbLink = RELAID_UR1toUR4;	
				}break;
				case 11:{							//����1������λ��RS485(UART3)͸��ת��
					configParam.IsUsbLink = RELAID_UR1autoUR3;
					gManage_t = 0xFE;
				}break;
				case 12:{							//����1������λ��BLE(UART2)͸��ת��
					configParam.IsUsbLink = RELAID_UR1autoUR2;
					gManage_t = 0xFE;
				}break;
				case UP_CMD_COPY_APP:{		   		//������Boot���������̼���App�в�����
					uart1SendTxQueueTimeout(&uart1Buff_t);
					vTaskDelay(500);
					SoftReset_fun();					
				}break;
				case UP_CMD_COPY_BOOT:{		   		//���������̼���Boot�в�����
					uart1SendTxQueueTimeout(&uart1Buff_t);
					vTaskDelay(500);					
					if(appFirCopyBoot()){	   					//�����ɹ�������������ʧ����ظ���λ��
						uart1Buff_t.DataBuff[PACKET_RESULT_INDEX]  = PACKET_ACK_OK;	
					}else{
						uart1Buff_t.DataBuff[PACKET_RESULT_INDEX]  = PACKET_ACK_GoToApp_ERROR;	
					}
					uart1Buff_t.DataBuff[PACKET_CMD_INDEX] = UP_BOOT_GOTO_APP;		
					uart1Buff_t.length_t = sendProcessing(uart1Buff_t.DataBuff,CMD_UPGRADER);
				}break;
				case UP_APP_GO_BOOT:{				//APP��ת��BOOT
					appSetUpFalfg();
					uart1SendTxQueueTimeout(&uart1Buff_t);
					vTaskDelay(500);
					SoftReset_fun();					
				}break;
				default:{
					switch(configParam.IsUsbLink){
						case RELAID_USBtoUR1:		//USB������λ��UART1ת��,��UART1���յ�����ͨ��USB��Э��ת����ȥ
						case RELAID_UR5toUR1:		//RS232(UART5)������λ��UART1ת��,��UART1���յ�����ͨ��RS232(UART5)��Э��ת����ȥ
						case RELAID_UR2toUR1:{		//BLE(UART2)������λ��UART1ת��,��UART1���յ�����ͨ��BLE(UART2)��Э��ת����ȥ
							memmove(&uart1Buff_t.DataBuff[PACK_DATA_INDEX],uart1Buff_t.DataBuff,uart1Buff_t.length_t);
							uart1Buff_t.DataBuff[PACK_LEN_H_INDEX] = uart1Buff_t.length_t>>8;
							uart1Buff_t.DataBuff[PACK_LEN_L_INDEX] = uart1Buff_t.length_t;
							uart1Buff_t.DataBuff[RESULTS_TYPE_INDEX] = RELAID_UART1;
							uart1Buff_t.length_t = sendProcessing(uart1Buff_t.DataBuff,CMD_PORT_RELAID);
							communicationSend_Struct(&uart1Buff_t,configParam.IsUsbLink);
						}break;
						case RELAID_UR1autoUR3:{	//UART1������λ��RS485(UART3)͸��ת��,��UART1���յ�����ͨ��RS485(UART3)ֱ��ת����ȥ
							communicationSend_Struct(&uart1Buff_t,RELAID_UR3toUR3);
//							communicationSend_Struct(&uart1Buff_t,RELAID_UR4toUR4);
						}break;
						case RELAID_UR1autoUR2:{	//UART1������λ��BLE(UART2)͸��ת��,��UART1���յ�����ͨ��BLE(UART2)ֱ��ת����ȥ
							communicationSend_Struct(&uart1Buff_t,UR2_Enabled);
						}break;
					}			
				}break;
			}
			#endif
			if(0xFE!=gManage_t){								//0XFE���ûظ�
				uart1SendTxQueueTimeout(&uart1Buff_t);
			}
		}
	}
}
static void uart1Tx_HTask(void *pvParameters)
{	
	stcATBuff uart1TxBuff_t;
    for( ;; ) {	
		uart1ReadTxQueueTimeout(&uart1TxBuff_t);
	}
}
void uart1_TaskInit()
{	
	uart1RecvQueue = xQueueCreate(5, sizeof(stcATBuff));
	if(configParam.ROS_SERIALx != SERIAL1){
		uart1SendQueue = xQueueCreate(5, sizeof(stcATBuff));
		uart1CompleteBinary = xSemaphoreCreateBinary();
		xSemaphoreTake(uart1CompleteBinary, 0);
		#ifndef Custom
		registerComSendCallback(uart1SendTxQueueTimeout,RELAID_UART1);
		#endif
		u1TaskSetDmaSendOver_func(uart1DmaSendOver_callback);			//DMA�������ע��
		u1SetDmaRead_func(uart1SendRxQueueTimeout);						//DMA��ȡ��������ע��
		USART1_Init(115200);
		xTaskCreate(uart1Tx_HTask,(const char *)"uart1Tx_HTask",256, NULL,uart1Tx_Pri, NULL);
	}else{
		#ifndef Custom
		rosSetuart1Taskfunc(uart1ToToolsDisable);
		rosSetDmaRead_func(uart1ToToolsSendRxQueueTimeout);
		#endif
	}
	xTaskCreate(uart1Rx_HTask,(const char *)"uart1Rx_HTask",256, NULL,uart1Rx_Pri, NULL);
}