#include "usart1.h"	
stcATBuff Uart1_RXbuff;
static void(*dmaRead_func)(stcATBuff *getStcABuff) = NULL;								//DMA接收数据回调函数
static volatile void (* uartDmaSendOver)(void) = NULL;       							//DMA发送完成回调函
static volatile void (* uartTaskDmaSendOver)(void) = NULL;       						//DMA发送完成回调函
static void USART1_DMA_Init(void);
void USART1_Init(uint32_t baudrate){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 								//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);								//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 							//GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 							//GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 							//GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 										//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 										//上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); 												//初始化PA9，PA10
	//USART1 初始化设置

	#ifdef SERIAL1_CONFIG
		(void)baudrate;
		getUARTConfigParam(&configParam.uart1Cfg,&USART_InitStructure);
	#else
		USART_InitStructure.USART_BaudRate = baudrate;									//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式
	#endif

	USART_Init(USART1, &USART_InitStructure); 											//初始化串口1
	USART_Cmd(USART1, ENABLE);  														//使能串口1 
	if(dmaRead_func!=NULL){
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);									//开启接收空闲中断
	}
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;									//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;								//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =6;									//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;										//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);														//根据指定的参数初始化VIC寄存器、	
	USART1_DMA_Init();
}
static void USART1_DMA_Init(void){
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);									//DMA2时钟使能      
	//****************************配置USART1发送  TX
	DMA_DeInit(DMA2_Stream7);                         									//DMA数据流
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);									//等待DMA可配置   
	// 配置 DMA Stream   
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  									//通道选择  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;						//目的：DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = NULL;										//源：DMA存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;								//方向：存储器到外设模式 
	DMA_InitStructure.DMA_BufferSize = 0;												//数据传输量   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//外设非增量模式  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//存储器增量模式  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;				//外设数据长度:8位  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						//存储器数据长度:8位  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;										//使用普通模式   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;								//中等优先级  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           					//FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;   					//FIFO大小
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;							//存储器突发单次传输  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;					//外设突发单次传输  
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);											//初始化DMA Stream  
	//DMA NVIC 
	#ifndef BOOTLOADER
	if(uartDmaSendOver!=NULL || uartTaskDmaSendOver!= NULL){	
	#endif
		DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);									//配置DMA发送完成后产生中断  	
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;  						//DMA中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =7;    
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
		NVIC_Init(&NVIC_InitStructure);
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  									//使能串口1的DMA发送
		DMA_Cmd(DMA2_Stream7, DISABLE);
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  									//使能串口1的DMA发送
	#ifndef BOOTLOADER
	}
	#endif
	//****************************配置USART1 RX
	if(dmaRead_func!=NULL){
		DMA_DeInit(DMA2_Stream5);  
		while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);								//等待DMA可配置   
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  								//通道选择  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;					//DMA外设地址  
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Uart1_RXbuff.DataBuff;				//DMA 存储器0地址  
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
		DMA_Init(DMA2_Stream5, &DMA_InitStructure);										//初始化DMA Stream 
		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  									//使能串口1的DMA接收			
		DMA_Cmd(DMA2_Stream5, ENABLE); 													//开启串口1的DMA接收传输
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
	DMA_Cmd(DMA2_Stream7,DISABLE); 														//禁用DMA Disable DMA
	DMA2_Stream7->M0AR = (uint32_t)buf; 												//设置拷贝的数据地址 set memory addr
	DMA2_Stream7->NDTR = length;														//设置拷贝的长度
	DMA_Cmd(DMA2_Stream7,ENABLE); 														//启动DMA
}
void u1_printf(char* fmt,...)  
{  	
	static stcATBuff strBuff_t;
	va_list ap;
	va_start(ap,fmt);
	strBuff_t.length_t = vsnprintf((char*)strBuff_t.DataBuff,strBuffLength,fmt,ap);
	va_end(ap);
	u1DataFrame_Send(strBuff_t.DataBuff,strBuff_t.length_t);							//DMA数据发送
}

void DMA2_Stream7_IRQHandler(void){	
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7)!= RESET) 								//检查DMA传输完成中断 DMA_IT_TCIF7	
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
void USART1_IRQHandler(void){                											//串口1中断服务程序 接收中断和发送中断
	if(USART_GetITStatus(USART1,USART_IT_IDLE))											//串口空闲加DMA接收数据
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

	
