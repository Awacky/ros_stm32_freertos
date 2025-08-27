#include "usart1.h"	
stcATBuff Uart1_RXbuff;
static void(*dmaRead_func)(stcATBuff *getStcABuff) = NULL;								//DMA�������ݻص�����
static volatile void (* uartDmaSendOver)(void) = NULL;       							//DMA������ɻص���
static volatile void (* uartTaskDmaSendOver)(void) = NULL;       						//DMA������ɻص���
static void USART1_DMA_Init(void);
void USART1_Init(uint32_t baudrate){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 								//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);								//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 							//GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 							//GPIOA10����ΪUSART1
	
	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 							//GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 										//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 										//����
	GPIO_Init(GPIOA,&GPIO_InitStructure); 												//��ʼ��PA9��PA10
	//USART1 ��ʼ������

	#ifdef SERIAL1_CONFIG
		(void)baudrate;
		getUARTConfigParam(&configParam.uart1Cfg,&USART_InitStructure);
	#else
		USART_InitStructure.USART_BaudRate = baudrate;									//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;							//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;								//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ
	#endif

	USART_Init(USART1, &USART_InitStructure); 											//��ʼ������1
	USART_Cmd(USART1, ENABLE);  														//ʹ�ܴ���1 
	if(dmaRead_func!=NULL){
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);									//�������տ����ж�
	}
	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;									//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;								//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =6;									//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;										//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);														//����ָ���Ĳ�����ʼ��VIC�Ĵ�����	
	USART1_DMA_Init();
}
static void USART1_DMA_Init(void){
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);									//DMA2ʱ��ʹ��      
	//****************************����USART1����  TX
	DMA_DeInit(DMA2_Stream7);                         									//DMA������
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);									//�ȴ�DMA������   
	// ���� DMA Stream   
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  									//ͨ��ѡ��  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;						//Ŀ�ģ�DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = NULL;										//Դ��DMA�洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;								//���򣺴洢��������ģʽ 
	DMA_InitStructure.DMA_BufferSize = 0;												//���ݴ�����   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//���������ģʽ  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//�洢������ģʽ  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;				//�������ݳ���:8λ  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						//�洢�����ݳ���:8λ  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;										//ʹ����ͨģʽ   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;								//�е����ȼ�  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           					//FIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;   					//FIFO��С
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;							//�洢��ͻ�����δ���  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;					//����ͻ�����δ���  
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);											//��ʼ��DMA Stream  
	//DMA NVIC 
	#ifndef BOOTLOADER
	if(uartDmaSendOver!=NULL || uartTaskDmaSendOver!= NULL){	
	#endif
		DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);									//����DMA������ɺ�����ж�  	
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;  						//DMA�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =7;    
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
		NVIC_Init(&NVIC_InitStructure);
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  									//ʹ�ܴ���1��DMA����
		DMA_Cmd(DMA2_Stream7, DISABLE);
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  									//ʹ�ܴ���1��DMA����
	#ifndef BOOTLOADER
	}
	#endif
	//****************************����USART1 RX
	if(dmaRead_func!=NULL){
		DMA_DeInit(DMA2_Stream5);  
		while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);								//�ȴ�DMA������   
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  								//ͨ��ѡ��  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;					//DMA�����ַ  
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Uart1_RXbuff.DataBuff;				//DMA �洢��0��ַ  
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;						//���赽�洢��ģʽ  
		DMA_InitStructure.DMA_BufferSize = strBuffLength;								//���ݴ�����   
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//���������ģʽ  
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;							//�洢������ģʽ  
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//�������ݳ���:8λ  
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//�洢�����ݳ���:8λ  
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;									//ʹ����ͨģʽ   
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;								//�е����ȼ�  
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;						//�洢��ͻ�����δ���  
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;				//����ͻ�����δ���  
		DMA_Init(DMA2_Stream5, &DMA_InitStructure);										//��ʼ��DMA Stream 
		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  									//ʹ�ܴ���1��DMA����			
		DMA_Cmd(DMA2_Stream5, ENABLE); 													//��������1��DMA���մ���
	}
}
void u1SetDmaSendOver_func(void (* const send)(void)){
	if(send == NULL) return;
	uartDmaSendOver = (void volatile(*)(void))send;
}
void u1TaskSetDmaSendOver_func(void (* const send)(void)){
	if(send == NULL) return;
	uartTaskDmaSendOver = (void volatile(*)(void))send;
}
void u1SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff)){
    dmaRead_func = dmaRead_func_t;
}
void u1IT_IDLE_ReadEnable(void){
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
}
void u1IT_IDLE_ReadDisable(void){
	USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
}
void u1DataFrame_Send(uint8_t *buf, uint16_t length){
	DMA_Cmd(DMA2_Stream7,DISABLE); 														//����DMA Disable DMA
	DMA2_Stream7->M0AR = (uint32_t)buf; 												//���ÿ��������ݵ�ַ set memory addr
	DMA2_Stream7->NDTR = length;														//���ÿ����ĳ���
	DMA_Cmd(DMA2_Stream7,ENABLE); 														//����DMA
}
void u1_printf(char* fmt,...)  
{  	
	static stcATBuff strBuff_t;
	va_list ap;
	va_start(ap,fmt);
	strBuff_t.length_t = vsnprintf((char*)strBuff_t.DataBuff,strBuffLength,fmt,ap);
	va_end(ap);
	u1DataFrame_Send(strBuff_t.DataBuff,strBuff_t.length_t);							//DMA���ݷ���
}

void DMA2_Stream7_IRQHandler(void){	
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7)!= RESET) 								//���DMA��������ж� DMA_IT_TCIF7	
	{
		int i=0;
		while(RESET == USART_GetFlagStatus(USART1,USART_FLAG_TC)){
			i++;if(i>0xffffff){break;}
		};
		if(uartDmaSendOver!=NULL){
			uartDmaSendOver();
		}
		if(uartTaskDmaSendOver != NULL){
			uartTaskDmaSendOver();
		}
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7); 
    }
}
void USART1_IRQHandler(void){                											//����1�жϷ������ �����жϺͷ����ж�
	if(USART_GetITStatus(USART1,USART_IT_IDLE))											//���ڿ��м�DMA��������
    {
		DMA_Cmd(DMA2_Stream5, DISABLE);		
		USART1->SR;
		USART1->DR;
		Uart1_RXbuff.length_t = strBuffLength - DMA_GetCurrDataCounter(DMA2_Stream5);
		DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 |
		DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);
		DMA_SetCurrDataCounter(DMA2_Stream5, strBuffLength);  
		DMA_Cmd(DMA2_Stream5, ENABLE);
		if(dmaRead_func!=NULL){
			dmaRead_func(&Uart1_RXbuff);
		}
    }
} 

	
