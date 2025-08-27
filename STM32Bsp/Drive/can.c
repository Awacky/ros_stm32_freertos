#ifdef __cplusplus
extern "C" {
#endif
	
#include "can.h"
	
static void(*osRead_func)(canStrMsg *getStcABuff) = 0;
static void CAN2_Cfg(uint8_t tsjw,uint8_t tbs1,uint8_t tbs2,uint16_t brp);	
//波特率=Fpclk1/((tsjw + tbs1 + tbs2)*brp);
//则波特率为:42M/((1+15+5)*2)=1Mbps	CIA=76.2%
//则波特率为:42M/((1+12+1)*6)=500k 	CIA=92.9%
//则波特率为:42M/((1+6+1)*21)=250K	CIA=87.5%
//则波特率为:42M/((1+6+1)*42)=125k 	CIA=87.5%
static void CAN2_Cfg(uint8_t tsjw,uint8_t tbs1,uint8_t tbs2,uint16_t brp)
{

	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   	NVIC_InitTypeDef  NVIC_InitStructure;

    //使能相关时钟
	RCC_AHB1PeriphClockCmd(STARBOT_CAN_TX_CLK, ENABLE);						   
	RCC_AHB1PeriphClockCmd(STARBOT_CAN_RX_CLK, ENABLE);					 	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);				
	RCC_APB1PeriphClockCmd(STARBOT_CAN_CLK, ENABLE);					
	
    //初始化GPIO
	GPIO_InitStructure.GPIO_Pin = STARBOT_CAN_TX_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//上拉
	GPIO_Init(STARBOT_CAN_TX_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = STARBOT_CAN_RX_GPIO;
    GPIO_Init(STARBOT_CAN_RX_GPIO_PORT, &GPIO_InitStructure);
	
	//引脚复用映射配置
	GPIO_PinAFConfig(STARBOT_CAN_TX_GPIO_PORT,STARBOT_CAN_TX_PINSOURCE,STARBOT_CAN_AF); 				//GPIOD0复用为CAN2 GPIO_AF_CAN1
	GPIO_PinAFConfig(STARBOT_CAN_RX_GPIO_PORT,STARBOT_CAN_RX_PINSOURCE,STARBOT_CAN_AF); 				//GPIOD1复用为CAN2
	
	CAN_DeInit(STARBOT_CAN);
	CAN_StructInit(&CAN_InitStructure);	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;									//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;									//软件自动离线管理	DISABLE  
  	CAN_InitStructure.CAN_AWUM=DISABLE;									//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;									//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;									//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;									//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	 						//模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;										//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; 									//Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;										//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  								//分频系数(Fdiv)为brp+1	
  	CAN_Init(STARBOT_CAN, &CAN_InitStructure);   								// 初始化CAN2 
    
	//配置过滤器
 	CAN_FilterInitStructure.CAN_FilterNumber=STARBOT_CAN_FilterNumber;	//过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 		//32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;					//32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;				//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;	//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; 				//激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);							//滤波器初始化
	
	CAN_ITConfig(STARBOT_CAN,CAN_IT_FMP0,ENABLE);						//FIFO0消息挂号中断允许.		    
  	NVIC_InitStructure.NVIC_IRQChannel = STARBOT_CAN_IRQ;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;    
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}   

////中断服务函数			    
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
	while((CAN_TransmitStatus(STARBOT_CAN, mbox)==CAN_TxStatus_Failed)&&(timerOut<0XFFF))timerOut++;	 	//等待发送结束
	if(timerOut>=0XFFF)return false;													 			//超时
	return true;												
	
}
void canSetosRead_func(void(*osRead_func_t)(canStrMsg *getStcABuff)){
    osRead_func = osRead_func_t;
}
#ifdef __cpluspluss
}
#endif



