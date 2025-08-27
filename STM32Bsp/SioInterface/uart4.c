#ifdef __cplusplus
extern "C" {
#endif
#include "uart4.h"		    
stcATBuff Uart4_RXbuff;
static void(*dmaRead_func)(stcATBuff *getStcABuff) = NULL;					//DMA�������ݻص�����
static volatile void (* uartDmaSendOver)(void) = NULL;       				//DMA������ɻص�����
static void UART4_DMA_Init(void);
void UART4_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 					//ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);					//ʹ��UART4ʱ�ӣ�ע�⴮��3��ʱ���ǹ���APB1�ϵ�

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;						//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 							//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 							//����
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); 

	//UART4 ��ʼ������
	#ifdef SERIAL4_CONFIG
		getUARTConfigParam(&configParam.uart4Cfg,&USART_InitStructure);
	#else
		USART_InitStructure.USART_BaudRate = baudrate;						//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;				//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;					//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//�շ�ģʽ
	#endif
	USART_Init(UART4, &USART_InitStructure); 								//��ʼ������4
	USART_Cmd(UART4, ENABLE);  												//ʹ�ܴ���4
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);							//����USART�����ж�
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;						//����4�ж�ͨ��  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;				//��ռ���ȼ�3  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;       				//�����ȼ�3  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         				//IRQͨ��ʹ��  
	NVIC_Init(&NVIC_InitStructure); 										//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	UART4_DMA_Init();
}
static void UART4_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);						//DMA1ʱ��ʹ�� 
	//****************************����UART4����  TX
	DMA_DeInit(DMA1_Stream4);  
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE);						//�ȴ�DMA������   
	/* ���� DMA Stream */  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  						//ͨ��ѡ��  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;				//DMA�����ַ  
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;								//DMA �洢��0��ַ  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//�洢��������ģʽ  
	DMA_InitStructure.DMA_BufferSize = strBuffLength;						//���ݴ�����   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//���������ģʽ  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//�洢������ģʽ  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//�������ݳ���:8λ  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�洢�����ݳ���:8λ  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//ʹ����ͨģʽ   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;					//�е����ȼ�  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//�洢��ͻ�����δ���  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//����ͻ�����δ���  
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);								//��ʼ��DMA Stream  
	
	//DMA NVIC
	if(uartDmaSendOver!=NULL){	    
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;    
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;    
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
		NVIC_Init(&NVIC_InitStructure);    
		DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);  
		USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  						//ʹ�ܴ���4��DMA����     
	}
	//****************************����UART1����   RX
	if(dmaRead_func!=NULL){
		DMA_DeInit(DMA1_Stream2);  
		while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);						//�ȴ�DMA������   
		
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  						//ͨ��ѡ��  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;				//DMA�����ַ  
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Uart4_RXbuff.DataBuff;		//DMA �洢��0��ַ  
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;				//���赽�洢��ģʽ  
		DMA_InitStructure.DMA_BufferSize = strBuffLength;						//���ݴ�����   
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//���������ģʽ  
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//�洢������ģʽ  
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//�������ݳ���:8λ  
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�洢�����ݳ���:8λ  
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//ʹ����ͨģʽ   
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//�е����ȼ�  
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//�洢��ͻ�����δ���  
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//����ͻ�����δ���  
		DMA_Init(DMA1_Stream2, &DMA_InitStructure);								//��ʼ��DMA Stream  
		USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);  							//ʹ�ܴ���4��DMA����
		DMA_Cmd(DMA1_Stream2, ENABLE);  										//��������4��DMA���մ���
	}
}
void u4SetDmaSendOver_func(void (* const send)(void)){
	if(send == NULL) return;
	uartDmaSendOver = (void volatile(*)(void))send;
}
void u4SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff)){
    dmaRead_func = dmaRead_func_t;
}
void u4IT_IDLE_ReadEnable(void){
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
}
void u4IT_IDLE_ReadDisable(void){
	USART_ITConfig(UART4, USART_IT_IDLE, DISABLE);
}
void u4DataFrame_Send(uint8_t *send_buf,uint16_t length)
{
	DMA_Cmd(DMA1_Stream4,DISABLE);
	DMA1_Stream4->M0AR = (uint32_t)send_buf;
	DMA1_Stream4->NDTR = length;
	DMA_Cmd(DMA1_Stream4,ENABLE); 
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
}
void u4_printf(char* fmt,...)  
{  
	stcATBuff strBuff_t;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)strBuff_t.DataBuff,fmt,ap);
	va_end(ap);
	strBuff_t.length_t=strlen((const char*)strBuff_t.DataBuff);
	u4DataFrame_Send(strBuff_t.DataBuff,strBuff_t.length_t);
}
void DMA1_Stream4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4) != RESET)
	{
		if(uartDmaSendOver!=NULL){
			uartDmaSendOver();
		}
		DMA_Cmd(DMA1_Stream4,DISABLE);
		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4); 
	}
}
void UART4_IRQHandler(void)
{    
    if(USART_GetITStatus(UART4,USART_IT_IDLE) != RESET)
    {
		DMA_Cmd(DMA1_Stream2, DISABLE); 
        UART4->SR;
        UART4->DR;
		Uart4_RXbuff.length_t = strBuffLength - DMA_GetCurrDataCounter(DMA1_Stream2);
        DMA_ClearFlag(DMA1_Stream2,DMA_FLAG_TCIF2 | DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2);//���DMA2_Steam7������ɱ�־  
		DMA_SetCurrDataCounter(DMA1_Stream2, strBuffLength);  
        DMA_Cmd(DMA1_Stream2, ENABLE);
		if(dmaRead_func!=NULL){
			dmaRead_func(&Uart4_RXbuff);
		}
    }
}
#ifdef __cplusplus
}
#endif

