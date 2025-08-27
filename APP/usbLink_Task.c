#include "usbLink_Task.h"
#include "config_param.h"
#include "usbd_cdc_vcp.h"
#include "hw_config.h"
#include "osQueueSemap.h"
#include "CustomPlot.h"
#include "modem.h"
//USB����ATKPPacket����
static xQueueHandle usbSendQueue;
static xQueueHandle usbReadQueue;
static xSemaphoreHandle usbCompleteBinary = NULL;
static void usbSendRxQueueTimeout(stcATBuff *buff_t){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(usbReadQueue, buff_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
bool usbReadRxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(usbReadQueue,buff_t, 1000) == pdTRUE)	/*����usbDataDelivery(1024������)��Ϣ*/
	{
		return true;
	}
	memset(buff_t->DataBuff,0,sizeof(buff_t->DataBuff));
	buff_t->length_t = 0;
	return false;
}
uint8_t usbSendTxQueueTimeout(void *buff_t)
{
	stcATBuff *pBuff_t = (stcATBuff *)buff_t;
	if (xQueueSend(usbSendQueue,pBuff_t,100) == pdTRUE)	/*����usbDataDelivery(1024������)��Ϣ*/
	{
		return true;
	}
	return false;
}
bool usbReadTxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(usbSendQueue,buff_t, 1) == pdTRUE)	/*����usbDataDelivery(1024������)��Ϣ*/
	{
		xSemaphoreTake(usbCompleteBinary, 0);
		taskENTER_CRITICAL();
		usbsendData(buff_t->DataBuff,buff_t->length_t);
		taskEXIT_CRITICAL();
			/* Wait the DMA send over & Start next */
		xSemaphoreTake(usbCompleteBinary, 1000);

		return true;
	}
	return false;
}
static void usbDmaSendOver_callback(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(usbCompleteBinary, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
void usbLinkTxTask(void *param)
{
	stcATBuff usbtx_p;
	while(1){
		usbReadTxQueueTimeout(&usbtx_p);
	}
}
//USB���⴮�ڽ���ATKPPacket����
void usblinkRxTask(void *param)
{
	stcATBuff usbrx_p;
	uint8_t gManage_t = 0;
	while(1){
		if (usbReadRxQueueTimeout(&usbrx_p)){
			#ifndef Custom
			gManage_t=replyProcessing(usbrx_p.DataBuff,(uint8_t)usbrx_p.DataBuff[PACK_CMD_INDEX],(uint16_t *)&(usbrx_p.length_t),0);
			switch(gManage_t){
				case 1:{										//�������
					configParamGiveSemaphore();
				}break;
				case 2:{										//�ָ�Ĭ�ϲ���
					restoreDefaultParam();
					configParamGiveSemaphore();
				}break;
				case 3:{										//ʹ��USB�����ϱ�
					configParam.IsUsbLink = USB_Enabled;	
				}break;
				case 4:{										//����USB�����ϱ�
					configParam.IsUsbLink = USB_Disable;	
				}break;
				case 5:{										//USB������λ��CAN����ת��
					configParam.IsUsbLink = RELAID_USBtoCAN;
				}break;
				case 6:{										//USB������λ��RS485(UART3)����ת��
					configParam.IsUsbLink = RELAID_USBtoUR3;
				}break;
				case 7:{										//USB������λ��RS232(UART5)����ת��
					configParam.IsUsbLink = RELAID_USBtoUR5;
				}break;
				case 8:{										//USB������λ��BLE(UART2)����ת��
					configParam.IsUsbLink = RELAID_USBtoUR2;
				}break;
				case 9:{										//USB������λ��UART4����ת��
					configParam.IsUsbLink = RELAID_USBtoUR4;
				}break;
				case 10:{										//USB������λ��UART1����ת��
					configParam.IsUsbLink = RELAID_USBtoUR1;
				}break;
				case 11:{										//USB������λ��RS485(UART3)͸��ת��
					configParam.IsUsbLink = RELAID_USBautoUR3;
					gManage_t = 0xFE;
				}break;
				case 12:{										//USB������λ��BLE(UART2)͸��ת��
					configParam.IsUsbLink = RELAID_USBautoUR2;
					gManage_t = 0xFE;
				}break;
				default:{
					switch(configParam.IsUsbLink){
						case RELAID_USBautoUR2:{				//USB������λ��BLE(UART2)͸��ת��,  ��USB���յ�����ͨ��BLE(UART2)ֱ��ת����ȥ
							communicationSend_Struct(&usbrx_p,UR2_Enabled);
						}break;
						case RELAID_USBautoUR3:{				//USB������λ��RS485(UART3)͸��ת��,��USB���յ�����ͨ��RS485(UART3)ֱ��ת����ȥ
							communicationSend_Struct(&usbrx_p,RELAID_UR3toUR3);
						}break;
					}
				}break;
			}
			#endif
			if(0xFE!=gManage_t){
				usbSendTxQueueTimeout(&usbrx_p);
			}
		}
	}
}
/*usb���ӳ�ʼ��*/
void usbLink_TaskInit()
{	
	if(configParam.ROS_SERIALx != USB_SIO){
		usbSendQueue = xQueueCreate(5, sizeof(stcATBuff));
		usbReadQueue = xQueueCreate(5, sizeof(stcATBuff));
		usbCompleteBinary = xSemaphoreCreateBinary();
		xSemaphoreTake(usbCompleteBinary, 0);
		setUSBSendOver_func(usbDmaSendOver_callback);				//DMA�������ע��
		usbSetDmaRead_func(usbSendRxQueueTimeout);
		#ifndef Custom
		registerComSendCallback(usbSendTxQueueTimeout,RELAID_USB);
		#endif		
		usbd_cdc_vcp_Init();
		xTaskCreate(usbLinkTxTask,(const char *)"usbLinkTxTask",256, NULL,usbLinkTx_Pri, NULL);
		xTaskCreate(usblinkRxTask,(const char *)"usblinkRxTask",256, NULL,usblinkRx_Pri, NULL);
	}
}


