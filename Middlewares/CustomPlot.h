#ifndef _CUSTOM_PLOT_H_
#define _CUSTOM_PLOT_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct _Color_
{
	uint8_t R_t;
	uint8_t G_t;
	uint8_t B_t;
}Color_str;
void CostomPlotInit(void);
void CostomPlot_Send(float src_t,uint8_t index,uint8_t Send_Status); 
#endif 



