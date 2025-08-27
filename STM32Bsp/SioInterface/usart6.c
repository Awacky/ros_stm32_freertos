#ifdef __cplusplus
extern "C" {
#endif

#include "usart6.h"		    
#include <stdarg.h>
#include "string.h"

u16 usart6_send_buf_length = 0;
u8 usart6_send_buf[USART6_RECEIVE_BUF_LENGTH];
u16 usart6_rx_buf_length = 0;
u8 usart6_rx_buf[USART6_RECEIVE_BUF_LENGTH];
	
u32 usart6_baudrate_buf[9] = {9600,38400,57600,100000,115200,256000,460800,921600,1382400};

static u32 USART6_BaudRate_Select(u8 index);

static u32 USART6_BaudRate_Select(u8 index)
{
	return  usart6_baudrate_buf[index];
}

/*
 * 函数名：USART6_Init
 * 描述  ：串口3初始化函数
 * 输入  ：波特率
 * 输出  ：无
 */
void USART6_Init(u8 baudrate_index)
{
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟，注意串口3的时钟是挂在APB1上的
	
	//USART6端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PA9，PA10
		//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART6); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART6); //GPIOA10复用为USART1

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = usart6_baudrate_buf[baudrate_index];//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART6, &USART_InitStructure); //初始化串口3
    USART_Cmd(USART6, ENABLE);  //使能串口3 
	
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//开启USART空闲中断
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级3  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //子优先级3  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能  
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
		
	//if(I_UWB_LPS_ROLE == TAG_ROLE)	usart_rx_length = I_UWB_LPS_TAG_DATAFRAME0_LENGTH;//设置接收数据长度 
	//else if(I_UWB_LPS_ROLE == AHRS_ROLE)	usart_rx_length = I_AHRS_DATAFRAME0_LENGTH;
	//usart1_rx_buf_length = 128;
	
	USART6_DMA_Init();
}


void USART6_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA1时钟使能 

	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送     
    USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);  //使能串口3的DMA接收
	
	//****************************配置USART6发送  
	DMA_DeInit(DMA1_Stream3);  
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);//等待DMA可配置   
	
	/* 配置 DMA Stream */  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA外设地址  
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)usart6_send_buf;//DMA 存储器0地址  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式  
	DMA_InitStructure.DMA_BufferSize = usart6_send_buf_length;//数据传输量   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输  
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);//初始化DMA Stream  
	
	//DMA NVIC    
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
	NVIC_Init(&NVIC_InitStructure);    

	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);  
	
	
	//****************************配置UART1接收  
	DMA_DeInit(DMA2_Stream1);  
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);//等待DMA可配置   
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA外设地址  
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)usart6_rx_buf;//DMA 存储器0地址  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;//外设到存储器模式  
	DMA_InitStructure.DMA_BufferSize = USART6_RECEIVE_BUF_LENGTH;//数据传输量   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式   
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//中等优先级  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输  
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);//初始化DMA Stream  
	
	DMA_Cmd(DMA2_Stream1, ENABLE);  //开启DMA传输
	
	usart6_rx_irq_updata_user_reset_status = 0;
	usart6_tx_irq_updata_user_reset_status = 0;
}


u8 usart6_tx_irq_updata_user_reset_status = 0;

void DMA2_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6) != RESET)
		{
			
        DMA_Cmd(DMA2_Stream6,DISABLE); //DISABLE DMA 
        
				DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF3); 
				usart6_tx_irq_updata_user_reset_status = 0;
    }
}

/*
 * Function Name:USART1_DataFrame_Send
 * Description	:
 * Input  			:send_buf, send_buf length
 * Output  			:None
 */ 
void USART6_DataFrame_Send(unsigned char *send_buf,int length)
{
	if(usart6_tx_irq_updata_user_reset_status == 0)
	{
		memcpy(&usart6_send_buf,send_buf,length);
	
		DMA_SetCurrDataCounter(DMA2_Stream6,length);//设置传输数据长度
		DMA_Cmd(DMA2_Stream6,ENABLE); //启动DMA
		USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);

		usart6_tx_irq_updata_user_reset_status = 1;
	}
}

u8 usart6_rx_irq_updata_user_reset_status = 0;

void USART6_IRQHandler(void)
{    
    if(USART_GetITStatus(USART6,USART_IT_IDLE) != RESET)
    {
				
				DMA_Cmd(DMA2_Stream1, DISABLE); 
        USART6->SR;
        USART6->DR;
        DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);//清除DMA2_Steam7传输完成标志  
				DMA_SetCurrDataCounter(DMA2_Stream1, USART6_RECEIVE_BUF_LENGTH);  
        DMA_Cmd(DMA2_Stream1, ENABLE);    
				usart6_rx_irq_updata_user_reset_status = 1;
    }
}
//串口3,printf 函数
//确保一次发送数据不超过USART6_MAX_SEND_LEN字节
void u6_printf(char* fmt,...)  
{  
	u16 i; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)usart6_send_buf,fmt,ap);
	va_end(ap);
	i=strlen((const char*)usart6_send_buf);		//此次发送数据的长度
	USART6_DataFrame_Send(usart6_send_buf,i); 
}	

#ifdef __cplusplus
}
#endif

