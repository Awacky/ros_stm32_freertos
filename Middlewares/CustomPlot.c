#include "CustomPlot.h"
#include "hw_config.h"
#include "modem.h"
#include "datatypes.h"
stcATBuff custBuff_t;
uint16_t  custIndex = PACK_DATA_INDEX;
void CostomPlotParamSet(uint8_t *buff_p,uint16_t *index,char *lineName,const Color_str color_t);
void CostomPlotInit(void){
	#ifndef Custom
	uint16_t buffIndex = PACK_DATA_INDEX;
	Color_str stColor;
	custBuff_t.DataBuff[buffIndex] = 5;//Dèòa????êy?Y?ú??μ?êyá?
	buffIndex++;
	stColor.R_t = 199;stColor.G_t = 21;stColor.B_t = 133;
	CostomPlotParamSet(custBuff_t.DataBuff,&buffIndex,"A",stColor);
	stColor.R_t = 199;stColor.G_t = 80;stColor.B_t = 133;
	CostomPlotParamSet(custBuff_t.DataBuff,&buffIndex,"B",stColor);
	stColor.R_t = 199;stColor.G_t = 120;stColor.B_t = 133;
	CostomPlotParamSet(custBuff_t.DataBuff,&buffIndex,"C",stColor);
	stColor.R_t = 199;stColor.G_t = 200;stColor.B_t = 133;
	CostomPlotParamSet(custBuff_t.DataBuff,&buffIndex,"D",stColor);
	stColor.R_t = 199;stColor.G_t = 255;stColor.B_t = 133;
	CostomPlotParamSet(custBuff_t.DataBuff,&buffIndex,"E",stColor);
	buffIndex -= PACK_DATA_INDEX;
	custBuff_t.DataBuff[PACK_LEN_H_INDEX] = buffIndex>>8;
	custBuff_t.DataBuff[PACK_LEN_L_INDEX] = buffIndex;
	custBuff_t.length_t = sendProcessing(custBuff_t.DataBuff,CMD_CUSTOM_PLOT_INIT);
	communicationSend_Struct(&custBuff_t,0);
	#endif
}
void CostomPlotParamSet(uint8_t *buff_p,uint16_t *index,char *lineName,const Color_str color_t){
	uint8_t bLen = 0;
	uint8_t bIndex = *index;
	bLen = 1;
	buff_p[bIndex] = bLen;bIndex++;							//名称长度
	memcpy(buff_p+bIndex,lineName,bLen);					//名称									
	bIndex += bLen;
	memcpy(buff_p+bIndex,&color_t,3);						//颜色
	bIndex +=3;
	buff_p[bIndex] = 1;										//线宽
	bIndex++;
	*index = bIndex;
}

void CostomPlot_Send(float src_t,uint8_t index,uint8_t Send_Status)  
{  
	#ifndef Custom
	// u8Buf		:数组地址
	// data 		:float转buff的值
	// Mode			:数据转换模式 1:float转buff,0:buff转float
	// bBigEndian	:编码模式 1大端数据，0小端数据
	// return		:Mode = 1-> 0;Mode = 1-> float
	custBuff_t.DataBuff[custIndex] = index;
	custIndex++;
	FloatMutualChar(&(custBuff_t.DataBuff[custIndex]),src_t,1,0);
	custIndex+=4;
	if(Send_Status){
		custIndex -= PACK_DATA_INDEX;
		custBuff_t.DataBuff[PACK_LEN_H_INDEX] = custIndex>>8;
		custBuff_t.DataBuff[PACK_LEN_L_INDEX] = custIndex;
		custIndex = PACK_DATA_INDEX;
		custBuff_t.length_t = sendProcessing(custBuff_t.DataBuff,CMD_CUSTOM_PLOT_SEND);
		communicationSend_Struct(&custBuff_t,0);
		memset(&custBuff_t,0,sizeof(stcATBuff));
	}
	#endif
}
