#include "canLink_Task.h"
#include "osQueueSemap.h"
#include "can.h"
#include "canMotor.h"

xQueueHandle canRecvQueue =NULL;
xQueueHandle canSendQueue =NULL;	

static void canSendRxQueueTimeout(canStrMsg *RxMessage_t)		//读取CAN接收队列数据
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(canRecvQueue, RxMessage_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static bool canReadRxQueueTimeout(canStrMsg *RxMessage_t)		//读取CAN接收队列数据
{
	if(xQueueReceive(canRecvQueue,RxMessage_t, 1000) == pdTRUE)	/*接收usbDataDelivery(1024个容量)消息*/
	{
		return true;
	}
	memset(RxMessage_t->Data,0,sizeof(RxMessage_t->Data));
	RxMessage_t->DLC = 0;
	return false;
}
static uint8_t canSendTxQueueTimeout(void *TxMessage)	//给CAN发送队列增加数据
{
	canStrMsg *pTxMes = (canStrMsg *)TxMessage;
	if (xQueueSend(canSendQueue,pTxMes,10) == pdTRUE)		/*接收usbDataDelivery(1024个容量)消息*/
	{
		return true;
	}
	return false;
}
static bool canReadTxQueueTimeout(canStrMsg *TxMessage)     	//在任务中读取CAN发送队列，并把数据发送出去
{	
	if (xQueueReceive(canSendQueue,TxMessage,5) == pdTRUE)		/*接收usbDataDelivery(1024个容量)消息*/
	{	
		return canComm_Transmit(TxMessage);	
	}
	return false;
}

void canLinkTxTask(void *param)
{
	canStrMsg TxMessage_p;
	while(1)
	{
		canReadTxQueueTimeout(&TxMessage_p);
	}
}
//CAN接收ATKPPacket任务
void canlinkRxTask(void *param)
{
	canStrMsg RxMessage_p;
	stcATBuff srcStrBuff;
	while(1){
		if(canReadRxQueueTimeout(&RxMessage_p)){	
			//USB连接上位机CAN转发,把CAN接收的数据通过USB带协议转发出去
			//BLE(UART2)连接上位机CAN转发,把CAN接收的数据通过BLE(UART2)带协议转发出去
			//RS232(UART5)连接上位机CAN转发,把CAN接收的数据通过RS232(UART5)带协议转发出去
			//UART1连接上位机CAN转发,把CAN接收的数据通过UART1带协议转发出去
			//ROS连接 CAN转发,把CAN接收的数据通过ROS带协议转发出去
			if( (RELAID_USBtoCAN == configParam.IsUsbLink) || \
				(RELAID_UR2toCAN == configParam.IsUsbLink) || \
				(RELAID_UR5toCAN == configParam.IsUsbLink) || \
				(RELAID_UR1toCAN == configParam.IsUsbLink) || \
				(3 == configParam.IsRosNodePub)){
				#ifndef Custom			
				srcStrBuff.length_t = PACK_DATA_INDEX;
				switch(RxMessage_p.IDE){
					case CAN_Id_Standard:{						//标准帧
						srcStrBuff.DataBuff[srcStrBuff.length_t] = 0;
						if(RxMessage_p.RTR == CAN_RTR_Remote){	//远程帧
							srcStrBuff.DataBuff[srcStrBuff.length_t++] |= 0x40;
						}else{									//数据帧
							srcStrBuff.DataBuff[srcStrBuff.length_t++] |= 0;
						}
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.StdId);
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.StdId)>>8;
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.StdId)>>16;
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.StdId)>>24;
					}
					break;
					case CAN_Id_Extended:{//扩展帧
						srcStrBuff.DataBuff[srcStrBuff.length_t] = 0;
						srcStrBuff.DataBuff[srcStrBuff.length_t] |= 0x80;
						if(RxMessage_p.RTR == CAN_RTR_Remote){	//远程帧
							srcStrBuff.DataBuff[srcStrBuff.length_t++] |= 0x40;
						}else{									//数据帧
							srcStrBuff.DataBuff[srcStrBuff.length_t++] |= 0;
						}
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.ExtId);
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.ExtId)>>8;
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.ExtId)>>16;
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.ExtId)>>24;
					}
					break;
				}
				srcStrBuff.DataBuff[srcStrBuff.length_t++] = RxMessage_p.DLC;
				memcpy(&srcStrBuff.DataBuff[srcStrBuff.length_t],RxMessage_p.Data,RxMessage_p.DLC);
				srcStrBuff.length_t += RxMessage_p.DLC;
				srcStrBuff.length_t -= PACK_DATA_INDEX;
				srcStrBuff.DataBuff[PACK_LEN_H_INDEX] = srcStrBuff.length_t>>8;
				srcStrBuff.DataBuff[PACK_LEN_L_INDEX] = srcStrBuff.length_t;
				srcStrBuff.DataBuff[RESULTS_TYPE_INDEX] = RELAID_CAN;
				srcStrBuff.length_t = sendProcessing(srcStrBuff.DataBuff,CMD_PORT_RELAID);
				if(configParam.IsRosNodePub==3){
					communicationSend_Struct(&srcStrBuff,RELAID_XXXtoROS);
				} else {
					communicationSend_Struct(&srcStrBuff,configParam.IsUsbLink);
				}
				#endif
			} else {
				canMotorFeedbackProcessing(&RxMessage_p);
			}
		}
	}
}
/*CAN初始化*/
void canLink_TaskInit(void)
{	
	canRecvQueue = xQueueCreate(5, sizeof(canStrMsg));
	canSendQueue = xQueueCreate(5, sizeof(canStrMsg));
	HwCan_Init();
	canSetosRead_func(canSendRxQueueTimeout);
	#ifndef Custom
	registerComSendCallback(canSendTxQueueTimeout,RELAID_CAN);	//发送
	registerComSendCallback(canComm_Transmit,RESERVE1);			//发送
	#endif
	xTaskCreate(canLinkTxTask,(const char *)"canLinkTxTask",128, NULL,canLinkTx_Pri, NULL);
	xTaskCreate(canlinkRxTask,(const char *)"canlinkRxTask",256, NULL,canlinkRx_Pri, NULL);
}