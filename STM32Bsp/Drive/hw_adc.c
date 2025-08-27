#include "hw_adc.h"

GPIO_TypeDef* ADC_PORT[ADC_n] = {STARBOT_ADC1_GPIO_PORT,STARBOT_M1CurrH_GPIO_PORT,STARBOT_M1CurrL_GPIO_PORT,\
								 STARBOT_M2CurrH_GPIO_PORT,STARBOT_M2CurrL_GPIO_PORT,STARBOT_M3CurrH_GPIO_PORT,STARBOT_M3CurrL_GPIO_PORT,\
								 STARBOT_M4Curr_GPIO_PORT};
uint16_t  ADC_PIN[ADC_n] = 		{STARBOT_ADC1_PIN,STARBOT_M1CurrH_PIN,STARBOT_M1CurrL_PIN,\
								 STARBOT_M2CurrH_PIN,STARBOT_M2CurrL_PIN,STARBOT_M3CurrH_PIN,STARBOT_M3CurrL_PIN,STARBOT_M4Curr_PIN};
uint32_t  ADC_PORT_CLK[ADC_n] = {STARBOT_ADC1_GPIO_CLK,STARBOT_M1CurrH_GPIO_CLK,STARBOT_M1CurrL_GPIO_CLK,\
								 STARBOT_M2CurrH_GPIO_CLK,STARBOT_M2CurrL_GPIO_CLK,STARBOT_M3CurrH_GPIO_CLK,STARBOT_M3CurrL_GPIO_CLK,\
								 STARBOT_M4Curr_GPIO_CLK};
uint16_t  ADC_CHANNEL[ADC_n] = {STARBOT_ADC1_CHANNEL,STARBOT_M1CurrH_CHANNEL,STARBOT_M1CurrL_CHANNEL,STARBOT_M2CurrH_CHANNEL,\
								STARBOT_M2CurrL_CHANNEL,STARBOT_M3CurrH_CHANNEL,STARBOT_M3CurrL_CHANNEL,STARBOT_M4Curr_CHANNEL};
//diagnostic trouble code
static float mag_AdcRat = ADC_Ratio;
__IO uint16_t ADC_ConvertedValue[ADC_n+1];//�̶���һ���ڲ�ͨ��	
//uint32_t ADC_CumulativeValue[ADC_n];
//float ADC_MeanValue[ADC_n];
//uint32_t SamplingFre = 0;
void Hw_ADC_Init(uint8_t Count_t,uint8_t mDriveType){
	uint8_t Name_t = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);                                        //ADC1ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);											//DMA2ʱ��ʹ�� 
	mag_AdcRat = ADC_Ratio;
	if(M_DCM_4WD == mDriveType){
		mag_AdcRat = ADC_Ratio_2;
		ADC_PORT[0] = STARBOT_ADC2_GPIO_PORT;
		ADC_PIN[0] = STARBOT_ADC2_PIN;
		ADC_PORT_CLK[0] = STARBOT_ADC2_GPIO_CLK;
		ADC_CHANNEL[0] = STARBOT_ADC2_CHANNEL;
	}
	for(Name_t=0;Name_t<Count_t;Name_t++){
		RCC_AHB1PeriphClockCmd(ADC_PORT_CLK[Name_t], ENABLE);
		GPIO_InitStructure.GPIO_Pin     = ADC_PIN[Name_t];
		GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  //����������
		GPIO_Init(ADC_PORT[Name_t], &GPIO_InitStructure);
	}
	DMA_DeInit(DMA2_Stream0);                         											//DMA������
	while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE);											//�ȴ�DMA������   
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  											//ͨ��ѡ��  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;						//DMA�����ַ  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_ConvertedValue;						//DMA �洢��0��ַ  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;									    //���䷽��Ϊ�����赽�ڴ�  
	DMA_InitStructure.DMA_BufferSize = Count_t+1;												//���ݴ�����   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				    		//���������ģʽ  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										//�洢������ģʽ  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;					//�������ݴ�СΪ���֣��������ֽ�   
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;							//�洢����СҲΪ���� 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												//ѭ������ģʽ 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;										  	//�����ȼ�  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;//DMA_FIFOMode_Disable;                //��ʹ��FIFO
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;                           //FIFO��ֵ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                                 //�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                         //����ͻ�����δ���
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);													//��ʼ��DMA Stream  
	DMA_Cmd(DMA2_Stream0, ENABLE); 
	
	ADC_TempSensorVrefintCmd(ENABLE);//ʹ���ڲ��¶ȴ�����
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;									//����ģʽ
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;									//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;               //���������׶�֮����ӳ�20��ʱ������
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 		            //DMAʧ��
	ADC_CommonInit(&ADC_CommonInitStructure);													//��ʼ��

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 									    //12λת������ 
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                                //ɨ��ģʽ���ڶ�ͨ��ת��
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                                          //����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;                 //�����ⲿ����
	ADC_InitStructure.ADC_ExternalTrigConv =  0;												//��ʹ����ҲҪ��ʼ��һ�£�����ڴ��Զ���������ֵ�����²���Ҫ�Ĵ���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                      				//�����Ҷ���      
	ADC_InitStructure.ADC_NbrOfConversion = Count_t+1;                                  			//ת��ͨ����  
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); 											//����DMA����һ�����ݺ󷢲�����DMA������ADCת������DMA�����޹�   
	ADC_DMACmd(ADC1, ENABLE);  
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16,1, ADC_SampleTime_480Cycles);  	 			//����ת��ͨ����˳��
	for(Name_t=0;Name_t<Count_t;Name_t++){
		ADC_RegularChannelConfig(ADC1,ADC_CHANNEL[Name_t],Name_t+2,ADC_SampleTime_480Cycles);	
	}

	ADC_SoftwareStartConv(ADC1); 
}
float getSamplingValue(uint8_t ChName){
	float value_ft;
	switch(ChName){
		case temp_Mcu:{
			value_ft = ((float)ADC_ConvertedValue[ChName]*3.3/4095 -0.76)*400 +25;
		}break;
		case voltage_Bus:{
			value_ft = (float)ADC_ConvertedValue[ChName]/mag_AdcRat;
		}break;
		case current_M1:{
			value_ft = ADC_ConvertedValue[2]+ADC_ConvertedValue[3];
		}break;
		case current_M2:{
			value_ft = ADC_ConvertedValue[4]+ADC_ConvertedValue[5];
		}break;
		case current_M3:{
			value_ft = ADC_ConvertedValue[6]+ADC_ConvertedValue[7];
		}break;
		case current_M4:{
			value_ft = ADC_ConvertedValue[8];
		}break;
		default:
			value_ft = ADC_ConvertedValue[ChName];
		break;
	}
	return value_ft;
}