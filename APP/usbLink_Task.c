#include "usbLink_Task.h"
#include "config_param.h"
#include "usbd_cdc_vcp.h"
#include "hw_config.h"
#include "osQueueSemap.h"
#include "CustomPlot.h"
#include "modem.h"
//USB发送ATKPPacket任务
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
	if (xQueueReceive(usbReadQueue,buff_t, 1000) == pdTRUE)	/*接收usbDataDelivery(1024个容量)消息*/
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
	if (xQueueSend(usbSendQueue,pBuff_t,100) == pdTRUE)	/*接收usbDataDelivery(1024个容量)消息*/
	{
		return true;
	}
	return false;
}
bool usbReadTxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(usbSendQueue,buff_t, 1) == pdTRUE)	/*接收usbDataDelivery(1024个容量)消息*/
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
//USB虚拟串口接收ATKPPacket任务
void usblinkRxTask(void *param)
{
	stcATBuff usbrx_p;
	uint8_t gManage_t = 0;
	while(1){
		if (usbReadRxQueueTimeout(&usbrx_p)){
			#ifndef Custom
			gManage_t=replyProcessing(usbrx_p.DataBuff,(uint8_t)usbrx_p.DataBuff[PACK_CMD_INDEX],(uint16_t *)&(usbrx_p.length_t),0);
			switch(gManage_t){
				case 1:{										//保存参数
					configParamGiveSemaphore();
				}break;
				case 2:{										//恢复默认参数
					restoreDefaultParam();
					configParamGiveSemaphore();
				}break;
				case 3:{										//使能USB主动上报
					configParam.IsUsbLink = USB_Enabled;	
				}break;
				case 4:{										//禁用USB主动上报
					configParam.IsUsbLink = USB_Disable;	
				}break;
				case 5:{										//USB连接上位机CAN数据转发
					configParam.IsUsbLink = RELAID_USBtoCAN;
				}break;
				case 6:{										//USB连接上位机RS485(UART3)数据转发
					configParam.IsUsbLink = RELAID_USBtoUR3;
				}break;
				case 7:{										//USB连接上位机RS232(UART5)数据转发
					configParam.IsUsbLink = RELAID_USBtoUR5;
				}break;
				case 8:{										//USB连接上位机BLE(UART2)数据转发
					configParam.IsUsbLink = RELAID_USBtoUR2;
				}break;
				case 9:{										//USB连接上位机UART4数据转发
					configParam.IsUsbLink = RELAID_USBtoUR4;
				}break;
				case 10:{										//USB连接上位机UART1数据转发
					configParam.IsUsbLink = RELAID_USBtoUR1;
				}break;
				case 11:{										//USB连接上位机RS485(UART3)透明转发
					configParam.IsUsbLink = RELAID_USBautoUR3;
					gManage_t = 0xFE;
				}break;
				case 12:{										//USB连接上位机BLE(UART2)透明转发
					configParam.IsUsbLink = RELAID_USBautoUR2;
					gManage_t = 0xFE;
				}break;
				default:{
					switch(configParam.IsUsbLink){
						case RELAID_USBautoUR2:{				//USB连接上位机BLE(UART2)透明转发,  把USB接收的数据通过BLE(UART2)直接转发出去
							communicationSend_Struct(&usbrx_p,UR2_Enabled);
						}break;
						case RELAID_USBautoUR3:{				//USB连接上位机RS485(UART3)透明转发,把USB接收的数据通过RS485(UART3)直接转发出去
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
/*usb连接初始化*/
void usbLink_TaskInit()
{	
	if(configParam.ROS_SERIALx != USB_SIO){
		usbSendQueue = xQueueCreate(5, sizeof(stcATBuff));
		usbReadQueue = xQueueCreate(5, sizeof(stcATBuff));
		usbCompleteBinary = xSemaphoreCreateBinary();
		xSemaphoreTake(usbCompleteBinary, 0);
		setUSBSendOver_func(usbDmaSendOver_callback);				//DMA发送完成注册
		usbSetDmaRead_func(usbSendRxQueueTimeout);
		#ifndef Custom
		registerComSendCallback(usbSendTxQueueTimeout,RELAID_USB);
		#endif		
		usbd_cdc_vcp_Init();
		xTaskCreate(usbLinkTxTask,(const char *)"usbLinkTxTask",256, NULL,usbLinkTx_Pri, NULL);
		xTaskCreate(usblinkRxTask,(const char *)"usblinkRxTask",256, NULL,usblinkRx_Pri, NULL);
	}
}


