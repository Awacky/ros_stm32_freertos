#ifdef __cplusplus
extern "C" {
#endif
#include "uart5.h"		
stcATBuff Uart5_RXbuff;
static void(*dmaRead_func)(stcATBuff *getStcABuff) = NULL;								//DMA�������ݻص�����
static volatile void (* uartDmaSendOver)(void) = NULL;       							//DMA������ɻص�����
static void UART5_DMA_Init(void);
void UART5_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	
	//UART5�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	//���ڶ�Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	

   //UART5 ��ʼ������
   	#ifdef SERIAL5_CONFIG
		getUARTConfigParam(&configParam.uart5Cfg,&USART_InitStructure);
	#else
		USART_InitStructure.USART_BaudRate = baudrate;									//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;							//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;								//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ
	#endif

	USART_Init(UART5, &USART_InitStructure); 											//��ʼ������5
	USART_Cmd(UART5, ENABLE);  															//ʹ�ܴ���5
	
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);										//����UART�����ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;									//����5�ж�ͨ��  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;							//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;       							//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         							//IRQͨ��ʹ��  
	NVIC_Init(&NVIC_InitStructure); 													//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	UART5_DMA_Init();
}


static void UART5_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);									//DMA1ʱ��ʹ�� 
	//****************************����UART5����  
	DMA_DeInit(DMA1_Stream7);  
	while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);									//�ȴ�DMA������   
	/* ���� DMA Stream */  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4; 	 									//ͨ��ѡ��  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART5->DR;							//DMA�����ַ  
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;											//DMA �洢��0��ַ  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;								//�洢��������ģʽ  
	DMA_InitStructure.DMA_BufferSize = strBuffLength;									//���ݴ�����   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//���������ģʽ  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//�洢������ģʽ  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;				//�������ݳ���:8λ  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						//�洢�����ݳ���:8λ  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;										//ʹ����ͨģʽ   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;								//�е����ȼ�  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;							//�洢��ͻ�����δ���  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;					//����ͻ�����δ���  
	DMA_Init(DMA1_Stream7, &DMA_InitStructure);											//��ʼ��DMA Stream  
	USART_DMACmd(UART5,USART_DMAReq_Tx,ENABLE);        									//ʹ�ܴ���5��DMA����  
	//DMA NVIC  
	if(uartDmaSendOver!=NULL){		
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;    
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;    
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
		NVIC_Init(&NVIC_InitStructure);    
		DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);  
	}
	//****************************����UART1����  
	if(dmaRead_func!=NULL){
		DMA_DeInit(DMA1_Stream0);  
		while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);								//�ȴ�DMA������   
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  								//ͨ��ѡ��  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART5->DR;						//DMA�����ַ  
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Uart5_RXbuff.DataBuff;				//DMA �洢��0��ַ  
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
		DMA_Init(DMA1_Stream0, &DMA_InitStructure);										//��ʼ��DMA Stream  
		USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE);  									//ʹ�ܴ���5��DMA����
		DMA_Cmd(DMA1_Stream0, ENABLE);  												//��������4��DMA���մ���
	}	   
}
void u5SetDmaSendOver_func(void (* const send)(void)){
	if(send == NULL) return;
	uartDmaSendOver = (void volatile(*)(void))send;
}
void u5SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff)){
    dmaRead_func = dmaRead_func_t;
}
void u5IT_IDLE_ReadEnable(void){
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
}
void u5IT_IDLE_ReadDisable(void){
	USART_ITConfig(UART4, USART_IT_IDLE, DISABLE);
}
void u5DataFrame_Send(uint8_t *send_buf,uint16_t length)
{
	DMA_Cmd(DMA1_Stream7,DISABLE);
	DMA1_Stream7->M0AR = (uint32_t)send_buf;
	DMA1_Stream7->NDTR = length;
	DMA_Cmd(DMA1_Stream7,ENABLE); 
	USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);
}
void u5_printf(char* fmt,...)  
{  
	stcATBuff strBuff_t;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)strBuff_t.DataBuff,fmt,ap);
	va_end(ap);
	strBuff_t.length_t=strlen((const char*)strBuff_t.DataBuff);
	u5DataFrame_Send(strBuff_t.DataBuff,strBuff_t.length_t);
}
void DMA1_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream7,DMA_IT_TCIF7) != RESET)
	{
		if(uartDmaSendOver!=NULL){
			uartDmaSendOver();
		}
		DMA_Cmd(DMA1_Stream7,DISABLE); 						 
		DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7); 
    }
}
void UART5_IRQHandler(void)
{    
    if(USART_GetITStatus(UART5,USART_IT_IDLE) != RESET)
    {
		DMA_Cmd(DMA1_Stream0, DISABLE); 
        UART5->SR;
        UART5->DR;
		Uart5_RXbuff.length_t = strBuffLength - DMA_GetCurrDataCounter(DMA1_Stream0);
        DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0 | DMA_FLAG_FEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_TEIF0 | DMA_FLAG_HTIF0);//���DMA2_Steam7������ɱ�־  
		DMA_SetCurrDataCounter(DMA1_Stream0, strBuffLength);  
        DMA_Cmd(DMA1_Stream0, ENABLE);    
		if(dmaRead_func!=NULL){
			dmaRead_func(&Uart5_RXbuff);
		}	
    }
}
#ifdef __cplusplus
}
#endif

