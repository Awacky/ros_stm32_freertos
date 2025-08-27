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
 * ��������USART6_Init
 * ����  ������3��ʼ������
 * ����  ��������
 * ���  ����
 */
void USART6_Init(u8 baudrate_index)
{
	//GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ�ӣ�ע�⴮��3��ʱ���ǹ���APB1�ϵ�
	
	//USART6�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��PA9��PA10
		//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART6); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART6); //GPIOA10����ΪUSART1

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = usart6_baudrate_buf[baudrate_index];//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART6, &USART_InitStructure); //��ʼ������3
    USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���3 
	
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//����USART�����ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����1�ж�ͨ��  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�3  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�3  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQͨ��ʹ��  
    NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���
		
	//if(I_UWB_LPS_ROLE == TAG_ROLE)	usart_rx_length = I_UWB_LPS_TAG_DATAFRAME0_LENGTH;//���ý������ݳ��� 
	//else if(I_UWB_LPS_ROLE == AHRS_ROLE)	usart_rx_length = I_AHRS_DATAFRAME0_LENGTH;
	//usart1_rx_buf_length = 128;
	
	USART6_DMA_Init();
}


void USART6_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA1ʱ��ʹ�� 

	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA����     
    USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);  //ʹ�ܴ���3��DMA����
	
	//****************************����USART6����  
	DMA_DeInit(DMA1_Stream3);  
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);//�ȴ�DMA������   
	
	/* ���� DMA Stream */  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA�����ַ  
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)usart6_send_buf;//DMA �洢��0��ַ  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ  
	DMA_InitStructure.DMA_BufferSize = usart6_send_buf_length;//���ݴ�����   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ   
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���  
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);//��ʼ��DMA Stream  
	
	//DMA NVIC    
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
	NVIC_Init(&NVIC_InitStructure);    

	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);  
	
	
	//****************************����UART1����  
	DMA_DeInit(DMA2_Stream1);  
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);//�ȴ�DMA������   
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA�����ַ  
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)usart6_rx_buf;//DMA �洢��0��ַ  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;//���赽�洢��ģʽ  
	DMA_InitStructure.DMA_BufferSize = USART6_RECEIVE_BUF_LENGTH;//���ݴ�����   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ   
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//�е����ȼ�  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���  
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);//��ʼ��DMA Stream  
	
	DMA_Cmd(DMA2_Stream1, ENABLE);  //����DMA����
	
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
	
		DMA_SetCurrDataCounter(DMA2_Stream6,length);//���ô������ݳ���
		DMA_Cmd(DMA2_Stream6,ENABLE); //����DMA
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
        DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);//���DMA2_Steam7������ɱ�־  
				DMA_SetCurrDataCounter(DMA2_Stream1, USART6_RECEIVE_BUF_LENGTH);  
        DMA_Cmd(DMA2_Stream1, ENABLE);    
				usart6_rx_irq_updata_user_reset_status = 1;
    }
}
//����3,printf ����
//ȷ��һ�η������ݲ�����USART6_MAX_SEND_LEN�ֽ�
void u6_printf(char* fmt,...)  
{  
	u16 i; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)usart6_send_buf,fmt,ap);
	va_end(ap);
	i=strlen((const char*)usart6_send_buf);		//�˴η������ݵĳ���
	USART6_DataFrame_Send(usart6_send_buf,i); 
}	

#ifdef __cplusplus
}
#endif

