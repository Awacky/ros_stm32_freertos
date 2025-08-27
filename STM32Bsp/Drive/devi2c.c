#include "devi2c.h"
#include "oled.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
#define I2C1_DR_Address	((uint32_t)0x40005410)
/*=====================================================================================================*/
/*=====================================================================================================*/
__IO uint8_t* I2C_ReadPtr;
extern __IO uint8_t Is_Init;
__IO uint16_t* I2C_WritePtr;
__IO uint32_t I2C_TimeCnt = I2C_TIME;
DMA_InitTypeDef DMA_InitStruct;
/*=====================================================================================================*/
/*=====================================================================================================*/
static uint8_t I2C_TimeOut(void){
	
}
void I2C1_Send_DMA_IRQ( void )
{ // IIC 发送中断函数  每次发送完都会产生一个中断
//	if(DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) != RESET) {
//		DMA_Cmd(DMA1_Stream6, DISABLE);
//		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);

		I2C_TimeCnt = I2C_TIME;
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF))
			if((I2C_TimeCnt--) == 0) I2C_TimeOut();

		I2C_GenerateSTOP(I2C1, ENABLE);
		*I2C_WritePtr = 0;
//	}
//	if(Is_Init != 0x10) // 判断当前是否需要继续开启刷新
//		OLED_Refreash();
}
/*=====================================================================================================*/
/*=====================================================================================================*/
static void I2C1_Recv_DMA_IRQ( void )
{
	if(DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) != RESET) {
		I2C_GenerateSTOP(I2C1, ENABLE);
		DMA_Cmd(DMA1_Stream0, DISABLE);
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
		*I2C_ReadPtr = 0;
	}
}
//void DMA1_Stream6_IRQHandler(){			//发送中断
//	I2C1_Send_DMA_IRQ();
//}

void DMA1_Stream0_IRQHandler(){			//接收中断
	I2C1_Recv_DMA_IRQ();
}
/*=====================================================================================================*/
/*=====================================================================================================*/

/*=====================================================================================================*/
/*=====================================================================================================*/
void DevHwI2C_Init(void){
	// 硬件IIC 和DMA初始化函数  引自网络  注释 ： Jeason
	GPIO_InitTypeDef GPIO_InitStruct;

	I2C_InitTypeDef I2C_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); //初始化必要的时钟

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);  // IO复用设置

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);// IO 基本参数初始化
	
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;      
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = I2C1_SPEED; //宏定义 IIC时钟速度  我的设置是 400k HZ
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &I2C_InitStruct);
}
void DevDmaI2C_Init(void){
	
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_FEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_TEIF6 |
								DMA_FLAG_HTIF6 | DMA_FLAG_TCIF6);
	DMA_Cmd(DMA1_Stream6, DISABLE);
	DMA_DeInit(DMA1_Stream6);
	DMA_InitStruct.DMA_Channel = DMA_Channel_1;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)0;// 暂时没有意义！
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = 0xFFFF; // 这个数据目前没有意义，实际使用会重新赋值
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6, &DMA_InitStruct);

	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_FEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_TEIF0 |
								DMA_FLAG_HTIF0 | DMA_FLAG_TCIF0);
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_DeInit(DMA1_Stream0);
	DMA_InitStruct.DMA_Channel = DMA_Channel_1;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)0;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = 0xFFFF;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream0, &DMA_InitStruct);

	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
	I2C_DMACmd(I2C1, ENABLE);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
uint32_t DevDmaI2c_Read( uint8_t* ReadBuf, uint8_t SlaveAddr, uint8_t ReadAddr, uint8_t* NumByte )
{
	I2C_ReadPtr = NumByte;
	
	I2C_TimeCnt = I2C_TIME;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_GenerateSTART(I2C1, ENABLE);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_SendData(I2C1, ReadAddr);

	I2C_TimeCnt = I2C_TIME;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF) == RESET)
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_GenerateSTART(I2C1, ENABLE);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Receiver);

	if((u16)(*NumByte) < 2) {
		I2C_TimeCnt = I2C_TIME;
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET)
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		I2C_AcknowledgeConfig(I2C1, DISABLE);
		(void)I2C1->SR2;

		I2C_GenerateSTOP(I2C1, ENABLE);

		I2C_TimeCnt = I2C_TIME;
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		*ReadBuf = I2C_ReceiveData(I2C1);

		(u16)(*NumByte)--;

		I2C_TimeCnt = I2C_TIME;
		while(I2C1->CR1 & I2C_CR1_STOP)
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		I2C_AcknowledgeConfig(I2C1, ENABLE);
	}else {
		I2C_TimeCnt = I2C_TIME;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		DMA_InitStruct.DMA_Channel = DMA_Channel_1;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)ReadBuf;
		DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStruct.DMA_BufferSize = (uint32_t)(*NumByte);
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream0, &DMA_InitStruct);

		I2C_DMALastTransferCmd(I2C1, ENABLE);

		DMA_Cmd(DMA1_Stream0, ENABLE);
	}

	I2C_TimeCnt = I2C_TIME;
	while(*NumByte > 0)
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	return SUCCESS;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
uint32_t DevDmaI2c_ReadReg( uint8_t* ReadBuf, uint8_t SlaveAddr, uint8_t ReadAddr, uint8_t NumByte )
{
	DevDmaI2c_Read(ReadBuf, SlaveAddr, ReadAddr, (uint8_t*)(&NumByte));
	I2C_TimeCnt = I2C_TIME;
	while(NumByte > 0)
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	return SUCCESS;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
uint32_t DevDmaI2c_Write( uint8_t* WriteBuf, uint8_t SlaveAddr, uint8_t WriteAddr, u16 NumByte )
{

	I2C_TimeCnt = I2C_TIME;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		if((I2C_TimeCnt--) == 0) return I2C_TimeOut();

	I2C_GenerateSTART(I2C1, ENABLE);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		if((I2C_TimeCnt--) == 0) return I2C_TimeOut();

	I2C_TimeCnt = I2C_TIME;
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		if((I2C_TimeCnt--) == 0) return I2C_TimeOut();

	I2C_SendData(I2C1, WriteAddr);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
		if((I2C_TimeCnt--) == 0) return I2C_TimeOut();

	DMA_InitStruct.DMA_Channel = DMA_Channel_1;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)WriteBuf;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = (uint32_t)(NumByte);
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6, &DMA_InitStruct);

	DMA_Cmd(DMA1_Stream6, ENABLE);

	return SUCCESS;
}

void DevHwI2C_WriteByte(uint8_t DevAddr,uint8_t RegAddr,uint8_t data)
{
	uint32_t wait_time=0;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)){
		wait_time++;
		if(wait_time>=I2C_TIME){
			wait_time=0;
			break;
		}
	}
	//发送起始信号
	I2C_GenerateSTART(I2C1, ENABLE);//开启I2C1
	
	//检测EV5事件
	wait_time=0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){/*EV5,主模式*/
		wait_time++;
		if(wait_time>=I2C_TIME){
			wait_time=0;
			break;
		}
	}
	
	//发送设备写地址
	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);	//器件地址 -- 默认0x78
	
	//检测EV6事件
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		wait_time++;
		if(wait_time>=I2C_TIME){
			wait_time=0;
			break;
		}
	}

	//发送要操作设备内部的地址
	I2C_SendData(I2C1, RegAddr);
	//检测EV8_2事件
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		wait_time++;
		if(wait_time>=I2C_TIME){
			wait_time=0;
			break;
		}
	}
	
	I2C_SendData(I2C1, data);//发送数据
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		wait_time++;
		if(wait_time>=I2C_TIME){
			wait_time=0;
			break;
		}
	}
	//发送停止信号
	I2C_GenerateSTOP(I2C1, ENABLE);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
uint32_t DevDmaI2c_WriteReg( uint8_t* WriteBuf, uint8_t SlaveAddr, uint8_t WriteAddr, u16 NumByte )
{
	DevDmaI2c_Write(WriteBuf, SlaveAddr, WriteAddr, NumByte);
	return SUCCESS;
}
