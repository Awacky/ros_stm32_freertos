#ifdef __cplusplus
extern "C" {
#endif
	
#include "can.h"
	
static void(*osRead_func)(canStrMsg *getStcABuff) = 0;
static void CAN2_Cfg(uint8_t tsjw,uint8_t tbs1,uint8_t tbs2,uint16_t brp);	
//������=Fpclk1/((tsjw + tbs1 + tbs2)*brp);
//������Ϊ:42M/((1+15+5)*2)=1Mbps	CIA=76.2%
//������Ϊ:42M/((1+12+1)*6)=500k 	CIA=92.9%
//������Ϊ:42M/((1+6+1)*21)=250K	CIA=87.5%
//������Ϊ:42M/((1+6+1)*42)=125k 	CIA=87.5%
static void CAN2_Cfg(uint8_t tsjw,uint8_t tbs1,uint8_t tbs2,uint16_t brp)
{

	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   	NVIC_InitTypeDef  NVIC_InitStructure;

    //ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(STARBOT_CAN_TX_CLK, ENABLE);						   
	RCC_AHB1PeriphClockCmd(STARBOT_CAN_RX_CLK, ENABLE);					 	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);				
	RCC_APB1PeriphClockCmd(STARBOT_CAN_CLK, ENABLE);					
	
    //��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = STARBOT_CAN_TX_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//����
	GPIO_Init(STARBOT_CAN_TX_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = STARBOT_CAN_RX_GPIO;
    GPIO_Init(STARBOT_CAN_RX_GPIO_PORT, &GPIO_InitStructure);
	
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(STARBOT_CAN_TX_GPIO_PORT,STARBOT_CAN_TX_PINSOURCE,STARBOT_CAN_AF); 				//GPIOD0����ΪCAN2 GPIO_AF_CAN1
	GPIO_PinAFConfig(STARBOT_CAN_RX_GPIO_PORT,STARBOT_CAN_RX_PINSOURCE,STARBOT_CAN_AF); 				//GPIOD1����ΪCAN2
	
	CAN_DeInit(STARBOT_CAN);
	CAN_StructInit(&CAN_InitStructure);	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;									//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;									//����Զ����߹���	DISABLE  
  	CAN_InitStructure.CAN_AWUM=DISABLE;									//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;									//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;									//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;									//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	 						//ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;										//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; 									//Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;										//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  								//��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(STARBOT_CAN, &CAN_InitStructure);   								// ��ʼ��CAN2 
    
	//���ù�����
 	CAN_FilterInitStructure.CAN_FilterNumber=STARBOT_CAN_FilterNumber;	//������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 		//32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;					//32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;				//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;	//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; 				//���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);							//�˲�����ʼ��
	
	CAN_ITConfig(STARBOT_CAN,CAN_IT_FMP0,ENABLE);						//FIFO0��Ϣ�Һ��ж�����.		    
  	NVIC_InitStructure.NVIC_IRQChannel = STARBOT_CAN_IRQ;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;    
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}   

////�жϷ�����			    
void STARBOT_CAN_RX_IRQHandler(void)
{	
	CanRxMsg RxMessage;
	if (CAN_GetITStatus(STARBOT_CAN,CAN_IT_FMP0)!= RESET)
	{
		CAN_Receive(STARBOT_CAN,0, &RxMessage);
		CAN_ClearITPendingBit(STARBOT_CAN, CAN_IT_FMP0);
		if(osRead_func){
			osRead_func((canStrMsg *)&RxMessage);
		}
	}
}

void HwCan_Init(void){
	#ifndef Custom
	getcanConfigParam(&configParam.canSetParam);
	#endif
	CAN2_Cfg(configParam.canSetParam.RSAW_t,configParam.canSetParam.BTS1_t,configParam.canSetParam.BTS2_t,configParam.canSetParam.BRP_t);
//	CAN2_Cfg(CAN_SJW_1tq,CAN_BS1_12tq,CAN_BS2_1tq,6);	
}
void canComm_Transmit_Sid(uint32_t id, uint8_t *data, uint8_t len){
	;
}
void canComm_Transmit_Eid(uint32_t id, uint8_t *data, uint8_t len){
	;
}	
uint8_t canComm_Transmit(void *Src){
	uint16_t mbox = 0;
	uint16_t timerOut = 0;
	CanTxMsg *pTxsg = (CanTxMsg *)Src;
	CAN_Transmit(STARBOT_CAN,pTxsg); 
	while((CAN_TransmitStatus(STARBOT_CAN, mbox)==CAN_TxStatus_Failed)&&(timerOut<0XFFF))timerOut++;	 	//�ȴ����ͽ���
	if(timerOut>=0XFFF)return false;													 			//��ʱ
	return true;												
	
}
void canSetosRead_func(void(*osRead_func_t)(canStrMsg *getStcABuff)){
    osRead_func = osRead_func_t;
}
#ifdef __cpluspluss
}
#endif



