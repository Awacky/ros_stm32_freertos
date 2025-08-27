#include "Hw_Spi.h"

void Hw_SPI_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);											//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(STARBOT_SPI_CLK, ENABLE);												//ʹ��SPI1ʱ��
	
	GPIO_InitStructure.GPIO_Pin = STARBOT_SPI_MISO_PIN|STARBOT_SPI_MOSI_PIN|STARBOT_SPI_SCLK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;													//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;													//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;												//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;													//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);															//��ʼ��

	GPIO_PinAFConfig(STARBOT_SPI_MISO_PORT,STARBOT_SPI_MISO_PS,STARBOT_SPI_G_AF); 											
	GPIO_PinAFConfig(STARBOT_SPI_MOSI_PORT,STARBOT_SPI_MOSI_PS,STARBOT_SPI_G_AF); 											
	GPIO_PinAFConfig(STARBOT_SPI_SCLK_PORT,STARBOT_SPI_SCLK_PS,STARBOT_SPI_G_AF); 											
	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB1PeriphClockCmd(STARBOT_SPI_CLK,ENABLE);													//��λSPI1
	RCC_APB1PeriphClockCmd(STARBOT_SPI_CLK,DISABLE);												//ֹͣ��λSPI1
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  							//����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;													//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;												//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;														//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;													//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;														//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;							//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;														//CRCֵ����Ķ���ʽ
	SPI_Init(STARBOT_SPI, &SPI_InitStructure);  													//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	SPI_Cmd(STARBOT_SPI, ENABLE); 																	//ʹ��SPI����
	SPI1_ReadWriteByte(0xff);//��������		
}
//SPI1�ٶ����ú���
//SPI�ٶ�=fAPB2/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2ʱ��һ��Ϊ84Mhz��
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
	STARBOT_SPI->CR1&=0XFFC7;//λ3-5���㣬�������ò�����
	STARBOT_SPI->CR1|=SPI_BaudRatePrescaler;	//����SPI1�ٶ� 
	SPI_Cmd(STARBOT_SPI,ENABLE); //ʹ��SPI1
} 
//SPI1 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		 			 
  while (SPI_I2S_GetFlagStatus(STARBOT_SPI, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������  
	SPI_I2S_SendData(STARBOT_SPI, TxData); //ͨ������SPIx����һ��byte  ����
  while (SPI_I2S_GetFlagStatus(STARBOT_SPI, SPI_I2S_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte  
	return SPI_I2S_ReceiveData(STARBOT_SPI); //����ͨ��SPIx������յ�����	
 		    
}