#ifdef __cplusplus
extern "C" {
#endif
	
#ifndef __CRC_CALC_H
#define __CRC_CALC_H
#include "stdio.h"	
#include <stdint.h>
	
uint16_t crc16_usb_calc(uint8_t *data, uint32_t length);
uint16_t calc_ubx_checksum(const uint8_t* buffer, const uint16_t length);	
	
#endif
#ifdef __cplusplus
}
#endif


