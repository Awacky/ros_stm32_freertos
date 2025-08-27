#ifdef __cplusplus
extern "C" {
#endif
#include "uart5.h"		
stcATBuff Uart5_RXbuff;
static void(*dmaRead_func)(stcATBuff *getStcABuff) = NULL;								//DMA接收数据回调函数
static volatile void (* uartDmaSendOver)(void) = NULL;       							//DMA发送完成回调函数
static void UART5_DMA_Init(void);
void UART5_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	
	//UART5端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	//串口对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	

   //UART5 初始化设置
   	#ifdef SERIAL5_CONFIG
		getUARTConfigParam(&configParam.uart5Cfg,&USART_InitStructure);
	#else
		USART_InitStructure.USART_BaudRate = baudrate;									//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式
	#endif

	USART_Init(UART5, &USART_InitStructure); 											//初始化串口5
	USART_Cmd(UART5, ENABLE);  															//使能串口5
	
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);										//开启UART空闲中断
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;									//串口5中断通道  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;							//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;       							//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         							//IRQ通道使能  
	NVIC_Init(&NVIC_InitStructure); 													//根据指定的参数初始化VIC寄存器
	UART5_DMA_Init();
}


static void UART5_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);									//DMA1时钟使能 
	//****************************配置UART5发送  
	DMA_DeInit(DMA1_Stream7);  
	while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);									//等待DMA可配置   
	/* 配置 DMA Stream */  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4; 	 									//通道选择  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART5->DR;							//DMA外设地址  
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;											//DMA 存储器0地址  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;								//存储器到外设模式  
	DMA_InitStructure.DMA_BufferSize = strBuffLength;									//数据传输量   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//外设非增量模式  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//存储器增量模式  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;				//外设数据长度:8位  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						//存储器数据长度:8位  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;										//使用普通模式   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;								//中等优先级  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;							//存储器突发单次传输  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;					//外设突发单次传输  
	DMA_Init(DMA1_Stream7, &DMA_InitStructure);											//初始化DMA Stream  
	USART_DMACmd(UART5,USART_DMAReq_Tx,ENABLE);        									//使能串口5的DMA发送  
	//DMA NVIC  
	if(uartDmaSendOver!=NULL){		
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;    
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;    
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
		NVIC_Init(&NVIC_InitStructure);    
		DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);  
	}
	//****************************配置UART1接收  
	if(dmaRead_func!=NULL){
		DMA_DeInit(DMA1_Stream0);  
		while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);								//等待DMA可配置   
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  								//通道选择  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART5->DR;						//DMA外设地址  
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Uart5_RXbuff.DataBuff;				//DMA 存储器0地址  
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;						//外设到存储器模式  
		DMA_InitStructure.DMA_BufferSize = strBuffLength;								//数据传输量   
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设非增量模式  
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;							//存储器增量模式  
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//外设数据长度:8位  
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//存储器数据长度:8位  
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;									//使用普通模式   
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;								//中等优先级  
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;						//存储器突发单次传输  
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;				//外设突发单次传输  
		DMA_Init(DMA1_Stream0, &DMA_InitStructure);										//初始化DMA Stream  
		USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE);  									//使能串口5的DMA接收
		DMA_Cmd(DMA1_Stream0, ENABLE);  												//开启串口4的DMA接收传输
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
        DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0 | DMA_FLAG_FEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_TEIF0 | DMA_FLAG_HTIF0);//清除DMA2_Steam7传输完成标志  
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

