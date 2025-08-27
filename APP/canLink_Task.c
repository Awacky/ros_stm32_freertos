#include "canLink_Task.h"
#include "osQueueSemap.h"
#include "can.h"
#include "canMotor.h"

xQueueHandle canRecvQueue =NULL;
xQueueHandle canSendQueue =NULL;	

static void canSendRxQueueTimeout(canStrMsg *RxMessage_t)		//��ȡCAN���ն�������
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(canRecvQueue, RxMessage_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static bool canReadRxQueueTimeout(canStrMsg *RxMessage_t)		//��ȡCAN���ն�������
{
	if(xQueueReceive(canRecvQueue,RxMessage_t, 1000) == pdTRUE)	/*����usbDataDelivery(1024������)��Ϣ*/
	{
		return true;
	}
	memset(RxMessage_t->Data,0,sizeof(RxMessage_t->Data));
	RxMessage_t->DLC = 0;
	return false;
}
static uint8_t canSendTxQueueTimeout(void *TxMessage)	//��CAN���Ͷ�����������
{
	canStrMsg *pTxMes = (canStrMsg *)TxMessage;
	if (xQueueSend(canSendQueue,pTxMes,10) == pdTRUE)		/*����usbDataDelivery(1024������)��Ϣ*/
	{
		return true;
	}
	return false;
}
static bool canReadTxQueueTimeout(canStrMsg *TxMessage)     	//�������ж�ȡCAN���Ͷ��У��������ݷ��ͳ�ȥ
{	
	if (xQueueReceive(canSendQueue,TxMessage,5) == pdTRUE)		/*����usbDataDelivery(1024������)��Ϣ*/
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
//CAN����ATKPPacket����
void canlinkRxTask(void *param)
{
	canStrMsg RxMessage_p;
	stcATBuff srcStrBuff;
	while(1){
		if(canReadRxQueueTimeout(&RxMessage_p)){	
			//USB������λ��CANת��,��CAN���յ�����ͨ��USB��Э��ת����ȥ
			//BLE(UART2)������λ��CANת��,��CAN���յ�����ͨ��BLE(UART2)��Э��ת����ȥ
			//RS232(UART5)������λ��CANת��,��CAN���յ�����ͨ��RS232(UART5)��Э��ת����ȥ
			//UART1������λ��CANת��,��CAN���յ�����ͨ��UART1��Э��ת����ȥ
			//ROS���� CANת��,��CAN���յ�����ͨ��ROS��Э��ת����ȥ
			if( (RELAID_USBtoCAN == configParam.IsUsbLink) || \
				(RELAID_UR2toCAN == configParam.IsUsbLink) || \
				(RELAID_UR5toCAN == configParam.IsUsbLink) || \
				(RELAID_UR1toCAN == configParam.IsUsbLink) || \
				(3 == configParam.IsRosNodePub)){
				#ifndef Custom			
				srcStrBuff.length_t = PACK_DATA_INDEX;
				switch(RxMessage_p.IDE){
					case CAN_Id_Standard:{						//��׼֡
						srcStrBuff.DataBuff[srcStrBuff.length_t] = 0;
						if(RxMessage_p.RTR == CAN_RTR_Remote){	//Զ��֡
							srcStrBuff.DataBuff[srcStrBuff.length_t++] |= 0x40;
						}else{									//����֡
							srcStrBuff.DataBuff[srcStrBuff.length_t++] |= 0;
						}
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.StdId);
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.StdId)>>8;
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.StdId)>>16;
						srcStrBuff.DataBuff[srcStrBuff.length_t++] = (RxMessage_p.StdId)>>24;
					}
					break;
					case CAN_Id_Extended:{//��չ֡
						srcStrBuff.DataBuff[srcStrBuff.length_t] = 0;
						srcStrBuff.DataBuff[srcStrBuff.length_t] |= 0x80;
						if(RxMessage_p.RTR == CAN_RTR_Remote){	//Զ��֡
							srcStrBuff.DataBuff[srcStrBuff.length_t++] |= 0x40;
						}else{									//����֡
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
/*CAN��ʼ��*/
void canLink_TaskInit(void)
{	
	canRecvQueue = xQueueCreate(5, sizeof(canStrMsg));
	canSendQueue = xQueueCreate(5, sizeof(canStrMsg));
	HwCan_Init();
	canSetosRead_func(canSendRxQueueTimeout);
	#ifndef Custom
	registerComSendCallback(canSendTxQueueTimeout,RELAID_CAN);	//����
	registerComSendCallback(canComm_Transmit,RESERVE1);			//����
	#endif
	xTaskCreate(canLinkTxTask,(const char *)"canLinkTxTask",128, NULL,canLinkTx_Pri, NULL);
	xTaskCreate(canlinkRxTask,(const char *)"canlinkRxTask",256, NULL,canlinkRx_Pri, NULL);
}