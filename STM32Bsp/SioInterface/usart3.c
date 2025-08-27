#ifdef __cplusplus
extern "C" {
#endif
#include "usart3.h"		    
stcATBuff Uart3_RXbuff;
static void(*dmaRead_func)(stcATBuff *getStcABuff) = NULL;						//DMA�������ݻص�����
static volatile void (* uartDmaSendOver)(void) = NULL;							//DMA������ɻص�����
static void USART3_DMA_Init(void);
void USART3_Init(uint32_t baudrate){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 						//ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);						//ʹ��USART3ʱ�ӣ�ע�⴮��3��ʱ���ǹ���APB1�ϵ�
	
	//USART3�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	//GPIO��ʼ������
	#ifdef ST_HV126
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;					
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					
		GPIO_Init(GPIOD, &GPIO_InitStructure);							
		GPIO_ResetBits(GPIOD,GPIO_Pin_15);
	#endif
	
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); 					//GPIO����ΪUSART3
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); 					//GPIO����ΪUSART3
    //USART3 ��ʼ������
	#ifdef SERIAL3_CONFIG
		getUARTConfigParam(&configParam.uart3Cfg,&USART_InitStructure);
	#else
		USART_InitStructure.USART_BaudRate = baudrate;							//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;				//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;					//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;						//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//�շ�ģʽ
	#endif
	USART_Init(USART3, &USART_InitStructure); 									//��ʼ������3
	USART_Cmd(USART3, ENABLE);  												//ʹ�ܴ���3 
	#ifdef ST_HV126
		USART_ITConfig(USART3, USART_IT_TC, ENABLE);  //����USART��������ж�
	#endif 
	USART_ClearFlag(USART3,USART_FLAG_TC);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);								//����USART�����ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;							//����1�ж�ͨ��  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;					//��ռ���ȼ�3  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       					//�����ȼ�3  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         					//IRQͨ��ʹ��  
	NVIC_Init(&NVIC_InitStructure); 											//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	USART3_DMA_Init();
}
static void USART3_DMA_Init(void){
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);							//DMA1ʱ��ʹ�� 
	//****************************����USART3����  
	DMA_DeInit(DMA1_Stream3);  
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);							//�ȴ�DMA������   
	
	/* ���� DMA Stream */  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  							//ͨ��ѡ��  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;				//DMA�����ַ  
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;									//DMA �洢��0��ַ  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;						//�洢��������ģʽ  
	DMA_InitStructure.DMA_BufferSize = strBuffLength;							//���ݴ�����   
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
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);									//��ʼ��DMA Stream  
	//DMA NVIC
	if(uartDmaSendOver!=NULL){	
		DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE); 	
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;    
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;    
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
		NVIC_Init(&NVIC_InitStructure);    
		USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  							//ʹ�ܴ���3��DMA����    
	}
	//****************************����UART3����  
	if(dmaRead_func!=NULL){
		DMA_DeInit(DMA1_Stream1);  
		while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);						//�ȴ�DMA������   
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  						//ͨ��ѡ��  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;			//DMA�����ַ  
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Uart3_RXbuff.DataBuff;		//DMA �洢��0��ַ  
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
		DMA_Init(DMA1_Stream1, &DMA_InitStructure);								//��ʼ��DMA Stream  
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);  							//ʹ�ܴ���3��DMA����
		DMA_Cmd(DMA1_Stream1, ENABLE);  										//��������3��DMA���մ���
	}
}
void u3SetDmaSendOver_func(void (* const send)(void)){
	if(send == NULL) return;
	uartDmaSendOver = (void volatile(*)(void))send;
}
void u3SetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff)){
    dmaRead_func = dmaRead_func_t;
}
void u3IT_IDLE_ReadEnable(void){
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
}
void u3IT_IDLE_ReadDisable(void){
	USART_ITConfig(USART3, USART_IT_IDLE, DISABLE);
}
void u3DataFrame_Send(uint8_t *send_buf,uint16_t length){
	#ifdef ST_HV126	
		GPIO_SetBits(GPIOD,GPIO_Pin_15);
	#endif
	DMA_Cmd(DMA1_Stream3,DISABLE);
	DMA1_Stream3->M0AR = (uint32_t)send_buf;
	DMA1_Stream3->NDTR = length;
	DMA_Cmd(DMA1_Stream3,ENABLE); 
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}
void u3_printf(char* fmt,...)  {  
	stcATBuff strBuff_t;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)strBuff_t.DataBuff,fmt,ap);
	va_end(ap);
	strBuff_t.length_t=strlen((const char*)strBuff_t.DataBuff);
	u3DataFrame_Send(strBuff_t.DataBuff,strBuff_t.length_t);
}	
void DMA1_Stream3_IRQHandler(void){
    if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3) != RESET){
		if(uartDmaSendOver!=NULL){
			uartDmaSendOver();
		}
		DMA_Cmd(DMA1_Stream3,DISABLE);
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3); 
    }
}
void USART3_IRQHandler(void){    
    if(USART_GetITStatus(USART3,USART_IT_IDLE) != RESET){	
		DMA_Cmd(DMA1_Stream1, DISABLE); 
        USART3->SR;
        USART3->DR;
		Uart3_RXbuff.length_t = strBuffLength - DMA_GetCurrDataCounter(DMA1_Stream1);
        DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | \
			DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);			
		DMA_SetCurrDataCounter(DMA1_Stream1,strBuffLength);  
        DMA_Cmd(DMA1_Stream1, ENABLE); 
		if(dmaRead_func!=NULL){
			dmaRead_func(&Uart3_RXbuff);
		}
		#ifdef ST_HV126
			GPIO_SetBits(GPIOD,GPIO_Pin_15);
		#endif 
    }
	#ifdef ST_HV126
		if(USART_GetITStatus(USART3,USART_IT_TC) != RESET){	
			GPIO_ResetBits(GPIOD,GPIO_Pin_15);
			USART_ClearITPendingBit(USART3,USART_IT_TC);
		}
	#endif 
}
#ifdef __cplusplus
}
#endif

