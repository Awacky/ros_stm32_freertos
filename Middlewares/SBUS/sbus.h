#ifndef _SBUS_H_
#define _SBUS_H_

#include <stdint.h>
typedef struct
{
	uint8_t  num_values;
	uint8_t  failsafe;
	uint8_t  frame_drop;
	uint8_t  max_values;
	uint16_t values[18];
}sbusCfg;
uint8_t sbus_decode(const uint8_t frame[25],sbusCfg *vaSrc);
#endif 





