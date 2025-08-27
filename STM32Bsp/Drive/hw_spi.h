
#ifndef _HW_SPI_H_
#define _HW_SPI_H_
	#ifdef __cplusplus
	extern "C" {
	#endif
	#include "hw_config.h"

	void Hw_SPI_Init(void);	
	void SPI1_SetSpeed(uint8_t SpeedSet); 			//设置SPI1速度   
	uint8_t SPI1_ReadWriteByte(uint8_t TxData);		//SPI1总线读写一个字节	
	#ifdef __cplusplus
	}
	#endif	
#endif

