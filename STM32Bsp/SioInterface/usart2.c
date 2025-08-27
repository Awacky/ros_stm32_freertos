#ifdef __cplusplus
extern "C" {
#endif
#include "devi2c.h"
#include "usart2.h"		    
stcATBuff Uart2_RXbuff;
static void(*dmaRead_func)(stcATBuff *getStcABuff) = NULL;						//DMA�������ݻص�����
static volatile void (* uartDmaSendOver)(void) = NULL;       					//DMA������ɻص�����
static void USART2_DMA_Init(void);
void USART2_Init(uint32_t baudrate)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 						//ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);						//ʹ��USART2ʱ�ӣ�ע�⴮��3��ʱ���ǹ���APB1�ϵ�
	
	//USART2�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6; 						//GPIOD5��GPIOD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;								//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;							//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 								//����
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
		//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
	//USART2 ��ʼ������	
	#ifdef SERIAL2_CONFIG
		getUARTConfigParam(&configParam.uart2Cfg,&USART_InitStructure);
	#else
		USART_InitStructure.USART_BaudRate = baudrate;							//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;				//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;					//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;						//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//�շ�ģʽ
	#endif
	USART_Init(USART2, &USART_InitStructure); 
	USART_Cmd(USART2, ENABLE); 
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);								//����USART�����ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;							//����2�ж�ͨ��  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;					//��ռ���ȼ�3  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;       					//�����ȼ�3  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         					//IRQͨ��ʹ��  
	NVIC_Init(&NVIC_InitStructure); 											//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	USART2_DMA_Init();
}

static void USART2_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);							//DMA1ʱ��ʹ�� 
	//****************************����USART2����  TX
	DMA_DeInit(DMA1_Stream6);                         							//DMA������
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);							//�ȴ�DMA������   
	// ���� DMA Stream   
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  							//ͨ��ѡ��  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;				//DMA�����ַ  
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;									//DMA �洢��0��ַ  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;						//�洢��������ģʽ  
	DMA_InitStructure.DMA_BufferSize = 0;										//���ݴ�����   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//���������ģʽ  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//�洢������ģʽ  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		//�������ݳ���:8λ  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				//�洢�����ݳ���:8λ  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;								//ʹ����ͨģʽ   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;						//�е����ȼ�  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					//�洢��ͻ�����δ���  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			//����ͻ�����δ���  
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);									//��ʼ��DMA Stream  
	//DMA NVIC  
	#ifndef BOOTLOADER
	if(uartDmaSendOver!=NULL){	
	#endif	
		DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;    
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;    
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
		NVIC_Init(&NVIC_InitStructure);    
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
		USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  							//ʹ�ܴ���2��DMA����
		DMA_Cmd(DMA1_Stream6, DISABLE);
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
		USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	#ifndef BOOTLOADER
	}
	#endif
	//RX 																			
	if(dmaRead_func!=NULL){
		DMA_DeInit(DMA1_Stream5);  
		while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);						//�ȴ�DMA������   
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  						//ͨ��ѡ��  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;			//DMA�����ַ  
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Uart2_RXbuff.DataBuff;		//DMA �洢��0��ַ  
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
		DMA_Init(DMA1_Stream5, &DMA_InitStructure);								//��ʼ��DMA Stream  
		USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);  							//ʹ�ܴ���2��DMA���� 		
		DMA_Cmd(DMA1_Stream5, ENABLE);											//��������2��DMA���մ���
	}
}
void u2SetDmaSendOver_func(void (* const send)(void)){
	if(send == NULL) return;
	uartDmaSendOver = (void volatile(*)(void))send;
}
void u2SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff)){
    dmaRead_func = dmaRead_func_t;
}
void u2IT_IDLE_ReadEnable(void){
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
}
void u2IT_IDLE_ReadDisable(void){
	USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);
}
void u2DataFrame_Send(uint8_t *send_buf,uint16_t length)
{	
	DMA_Cmd(DMA1_Stream6,DISABLE);
	DMA1_Stream6->M0AR = (uint32_t)send_buf;
	DMA1_Stream6->NDTR = length;
	DMA_Cmd(DMA1_Stream6,ENABLE); 
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}
void u2_printf(char* fmt,...)  
{  
	static stcATBuff strBuff_t;
	va_list ap;
	va_start(ap,fmt);
	strBuff_t.length_t = vsnprintf((char*)strBuff_t.DataBuff,strBuffLength,fmt,ap);
	va_end(ap);
	u2DataFrame_Send(strBuff_t.DataBuff,strBuff_t.length_t);
}	
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) != RESET)
	{
		#ifndef BOOTLOADER
			I2C1_Send_DMA_IRQ();
		#endif
		if(uartDmaSendOver!=NULL){
			uartDmaSendOver();
		}
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6); 
		DMA_Cmd(DMA1_Stream6,DISABLE);
    }
}
void USART2_IRQHandler(void)
{    
    if(USART_GetITStatus(USART2,USART_IT_IDLE))
    {
		DMA_Cmd(DMA1_Stream5, DISABLE);
		USART2->SR;
		USART2->DR;
		Uart2_RXbuff.length_t = strBuffLength - DMA_GetCurrDataCounter(DMA1_Stream5);
		DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 |
		DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);
		DMA_SetCurrDataCounter(DMA1_Stream5, strBuffLength);  
		DMA_Cmd(DMA1_Stream5, ENABLE);
		if(dmaRead_func!=NULL){
			dmaRead_func(&Uart2_RXbuff);
		}
    }
}


#ifdef __cplusplus
}
#endif

