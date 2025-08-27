#ifndef __DEVI2C_H
#define __DEVI2C_H

#include "stm32f4xx.h"

/*=====================================================================================================*/
/*=====================================================================================================*/
#define I2C_TIME	((uint32_t)655350)    	// IIC超时时间定义
#define I2C1_SPEED	((uint32_t)400000)   	// IIC 总线时钟频率  会影响到刷新速度
/*=====================================================================================================*/
/*=====================================================================================================*/
void DevHwI2C_Init(void);
void DevDmaI2C_Init(void);
uint32_t DevDmaI2c_Read( uint8_t* ReadBuf, uint8_t SlaveAddr, uint8_t ReadAddr, uint8_t* NumByte );
uint32_t DevDmaI2c_ReadReg( uint8_t* ReadBuf, uint8_t SlaveAddr, uint8_t ReadAddr, uint8_t NumByte );
uint32_t DevDmaI2c_Write( uint8_t* WriteBuf, uint8_t SlaveAddr, uint8_t WriteAddr, u16 NumByte );
    void DevHwI2C_WriteByte(uint8_t DevAddr,uint8_t RegAddr,uint8_t data);
uint32_t DevDmaI2c_WriteReg( uint8_t* WriteBuf, uint8_t SlaveAddr, uint8_t WriteAddr, u16 NumByte );

void I2C1_Send_DMA_IRQ( void );
/*=====================================================================================================*/
/*=====================================================================================================*/

#endif



