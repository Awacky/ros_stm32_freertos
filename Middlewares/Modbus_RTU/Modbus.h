#ifdef __cplusplus
extern "C" {
#endif
	
#ifndef __RS485_H
#define __RS485_H

#include "datatypes.h"

uint16_t MB_CRC16(uint8_t *buf,  uint16_t len);
void registeMBSend(uint8_t(*osSend_func_t)(void *getStcABuff));	
void MB_ReadCoil_01H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_WriteCoil_05H(uint8_t _addr, uint16_t _reg, uint16_t _sta);	
void MB_ReadInput_02H(uint8_t _addr, uint16_t _reg, uint16_t _num);	
void MB_ReadHoldingReg_03H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_ReadInputReg_04H(uint8_t _addr, uint16_t _reg, uint16_t _num);	
void MB_WriteHoldingReg_06H(uint8_t _addr, uint16_t _reg, uint16_t _data);
void MB_WriteNumHoldingReg_10H(uint8_t _addr, uint16_t _reg, uint16_t _num,uint8_t *_databuf);	

#endif
#ifdef __cplusplus
}
#endif


