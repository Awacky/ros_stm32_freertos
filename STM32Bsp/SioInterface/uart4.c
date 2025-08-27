#ifdef __cplusplus
extern "C" {
#endif
#include "uart4.h"		    
stcATBuff Uart4_RXbuff;
static void(*dmaRead_func)(stcATBuff *getStcABuff) = NULL;					//DMA接收数据回调函数
static volatile void (* uartDmaSendOver)(void) = NULL;       				//DMA发送完成回调函数
static void UART4_DMA_Init(void);
void UART4_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 					//使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);					//使能UART4时钟，注意串口3的时钟是挂在APB1上的

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;						//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 							//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 							//上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); 

	//UART4 初始化设置
	#ifdef SERIAL4_CONFIG
		getUARTConfigParam(&configParam.uart4Cfg,&USART_InitStructure);
	#else
		USART_InitStructure.USART_BaudRate = baudrate;						//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;				//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;					//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//收发模式
	#endif
	USART_Init(UART4, &USART_InitStructure); 								//初始化串口4
	USART_Cmd(UART4, ENABLE);  												//使能串口4
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);							//开启USART空闲中断
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;						//串口4中断通道  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;				//抢占优先级3  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;       				//子优先级3  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         				//IRQ通道使能  
	NVIC_Init(&NVIC_InitStructure); 										//根据指定的参数初始化VIC寄存器

	UART4_DMA_Init();
}
static void UART4_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);						//DMA1时钟使能 
	//****************************配置UART4发送  TX
	DMA_DeInit(DMA1_Stream4);  
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE);						//等待DMA可配置   
	/* 配置 DMA Stream */  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  						//通道选择  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;				//DMA外设地址  
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;								//DMA 存储器0地址  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//存储器到外设模式  
	DMA_InitStructure.DMA_BufferSize = strBuffLength;						//数据传输量   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设非增量模式  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//使用普通模式   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;					//中等优先级  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输  
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);								//初始化DMA Stream  
	
	//DMA NVIC
	if(uartDmaSendOver!=NULL){	    
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;    
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;    
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
		NVIC_Init(&NVIC_InitStructure);    
		DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);  
		USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  						//使能串口4的DMA发送     
	}
	//****************************配置UART1接收   RX
	if(dmaRead_func!=NULL){
		DMA_DeInit(DMA1_Stream2);  
		while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);						//等待DMA可配置   
		
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  						//通道选择  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;				//DMA外设地址  
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Uart4_RXbuff.DataBuff;		//DMA 存储器0地址  
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;				//外设到存储器模式  
		DMA_InitStructure.DMA_BufferSize = strBuffLength;						//数据传输量   
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设非增量模式  
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式  
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位  
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位  
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//使用普通模式   
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//中等优先级  
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输  
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输  
		DMA_Init(DMA1_Stream2, &DMA_InitStructure);								//初始化DMA Stream  
		USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);  							//使能串口4的DMA接收
		DMA_Cmd(DMA1_Stream2, ENABLE);  										//开启串口4的DMA接收传输
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
        DMA_ClearFlag(DMA1_Stream2,DMA_FLAG_TCIF2 | DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2);//清除DMA2_Steam7传输完成标志  
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

