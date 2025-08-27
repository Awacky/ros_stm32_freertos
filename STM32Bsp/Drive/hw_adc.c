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
__IO uint16_t ADC_ConvertedValue[ADC_n+1];//固定加一个内部通道	
//uint32_t ADC_CumulativeValue[ADC_n];
//float ADC_MeanValue[ADC_n];
//uint32_t SamplingFre = 0;
void Hw_ADC_Init(uint8_t Count_t,uint8_t mDriveType){
	uint8_t Name_t = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);                                        //ADC1时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);											//DMA2时钟使能 
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
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  //不带上下拉
		GPIO_Init(ADC_PORT[Name_t], &GPIO_InitStructure);
	}
	DMA_DeInit(DMA2_Stream0);                         											//DMA数据流
	while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE);											//等待DMA可配置   
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  											//通道选择  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;						//DMA外设地址  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_ConvertedValue;						//DMA 存储器0地址  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;									    //传输方向为从外设到内存  
	DMA_InitStructure.DMA_BufferSize = Count_t+1;												//数据传输量   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				    		//外设非增量模式  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										//存储器增量模式  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;					//外设数据大小为半字，即两个字节   
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;							//存储器大小也为半字 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												//循环传输模式 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;										  	//高优先级  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;//DMA_FIFOMode_Disable;                //不使用FIFO
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;                           //FIFO阈值
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                                 //存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                         //外设突发单次传输
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);													//初始化DMA Stream  
	DMA_Cmd(DMA2_Stream0, ENABLE); 
	
	ADC_TempSensorVrefintCmd(ENABLE);//使能内部温度传感器
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;									//独立模式
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;									//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;               //两个采样阶段之间的延迟20个时钟周期
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 		            //DMA失能
	ADC_CommonInit(&ADC_CommonInitStructure);													//初始化

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 									    //12位转换精度 
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                                //扫描模式用于多通道转换
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                                          //连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;                 //不用外部触发
	ADC_InitStructure.ADC_ExternalTrigConv =  0;												//即使不用也要初始化一下，免得内存自动带入错误的值，导致不必要的错误
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                      				//数据右对齐      
	ADC_InitStructure.ADC_NbrOfConversion = Count_t+1;                                  			//转换通道数  
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); 											//控制DMA传输一组数据后发不发送DMA请求，与ADC转换后发送DMA请求无关   
	ADC_DMACmd(ADC1, ENABLE);  
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16,1, ADC_SampleTime_480Cycles);  	 			//设置转换通道的顺序
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