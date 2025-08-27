#ifdef __cplusplus
extern "C" {
#endif
#include "devi2c.h"
#include "usart2.h"		    
stcATBuff Uart2_RXbuff;
static void(*dmaRead_func)(stcATBuff *getStcABuff) = NULL;						//DMA接收数据回调函数
static volatile void (* uartDmaSendOver)(void) = NULL;       					//DMA发送完成回调函数
static void USART2_DMA_Init(void);
void USART2_Init(uint32_t baudrate)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 						//使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);						//使能USART2时钟，注意串口3的时钟是挂在APB1上的
	
	//USART2端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6; 						//GPIOD5与GPIOD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;								//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;							//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 								//上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
		//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
	//USART2 初始化设置	
	#ifdef SERIAL2_CONFIG
		getUARTConfigParam(&configParam.uart2Cfg,&USART_InitStructure);
	#else
		USART_InitStructure.USART_BaudRate = baudrate;							//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;				//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;					//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;						//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//收发模式
	#endif
	USART_Init(USART2, &USART_InitStructure); 
	USART_Cmd(USART2, ENABLE); 
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);								//开启USART空闲中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;							//串口2中断通道  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;					//抢占优先级3  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;       					//子优先级3  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         					//IRQ通道使能  
	NVIC_Init(&NVIC_InitStructure); 											//根据指定的参数初始化VIC寄存器
	USART2_DMA_Init();
}

static void USART2_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);							//DMA1时钟使能 
	//****************************配置USART2发送  TX
	DMA_DeInit(DMA1_Stream6);                         							//DMA数据流
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);							//等待DMA可配置   
	// 配置 DMA Stream   
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  							//通道选择  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;				//DMA外设地址  
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;									//DMA 存储器0地址  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;						//存储器到外设模式  
	DMA_InitStructure.DMA_BufferSize = 0;										//数据传输量   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//存储器增量模式  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		//外设数据长度:8位  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				//存储器数据长度:8位  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;								//使用普通模式   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;						//中等优先级  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					//存储器突发单次传输  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			//外设突发单次传输  
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);									//初始化DMA Stream  
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
		USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  							//使能串口2的DMA发送
		DMA_Cmd(DMA1_Stream6, DISABLE);
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
		USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	#ifndef BOOTLOADER
	}
	#endif
	//RX 																			
	if(dmaRead_func!=NULL){
		DMA_DeInit(DMA1_Stream5);  
		while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);						//等待DMA可配置   
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  						//通道选择  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;			//DMA外设地址  
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Uart2_RXbuff.DataBuff;		//DMA 存储器0地址  
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
		DMA_Init(DMA1_Stream5, &DMA_InitStructure);								//初始化DMA Stream  
		USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);  							//使能串口2的DMA接收 		
		DMA_Cmd(DMA1_Stream5, ENABLE);											//开启串口2的DMA接收传输
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

