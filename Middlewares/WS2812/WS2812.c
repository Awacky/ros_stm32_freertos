#ifdef __cplusplus
extern "C" {
#endif

#include "WS2812.h"
#define TIMING_ONE  61	// 0.8us电平时间 580ns~1us
#define TIMING_ZERO 28	// 0.3us电平时间 220ns~380ns
#define BuffLength  10*24

static void(*wsDelay_func)(uint16_t daley_t) = NULL;											//延时函数
static void(*wsLedInit_func)(uint8_t LedType) = NULL;											//LED1初始化函数
static void(*wsLedSend_func)(uint8_t LedType,uint16_t *sendBuff,uint16_t buffLen) = NULL;		//LED2初始化函数

uint16_t dmaBuffer[BuffLength];
uint8_t ColorBuff[3]  = {0,0,0};
uint8_t BlackColor[3] = {0,0,0};

void RegistrationWs2812Fun(void(*wsDelay_func_t)(uint16_t daley_t),void(*wsLedInit_func_t)(uint8_t LedType),\
						   void(*wsLedSend_func_t)(uint8_t LedType,uint16_t *sendBuff,uint16_t buffLen)){
	wsDelay_func = wsDelay_func_t;
	wsLedInit_func = wsLedInit_func_t;
    wsLedSend_func = wsLedSend_func_t;							 
}
void WS2812_Init(uint8_t LedType){
	wsLedInit_func(LedType);
}
static void GetColorPix(uint8_t *OutColorBuff,uint16_t Color){
	switch(Color){
		case 1:{//红
			OutColorBuff[0] = 255;	//R
			OutColorBuff[1] = 0;	//G
			OutColorBuff[2] = 0;	//B
		}break;
		case 2:{//绿
			OutColorBuff[0] = 0;	//R
			OutColorBuff[1] = 255;	//G
			OutColorBuff[2] = 0;	//B
		}break;
		case 3:{//蓝
			OutColorBuff[0] = 0;	//R
			OutColorBuff[1] = 0;	//G
			OutColorBuff[2] = 255;	//B
		}break;
		case 4:{//白
			OutColorBuff[0] = 255;	//R
			OutColorBuff[1] = 255;	//G
			OutColorBuff[2] = 255;	//B
		}break;
		case 5:{//橙
			OutColorBuff[0] = 255;	//R
			OutColorBuff[1] = 153;	//G
			OutColorBuff[2] = 102;	//B
		}break;
		case 6:{//黄
			OutColorBuff[0] = 255;	//R
			OutColorBuff[1] = 127;	//G
			OutColorBuff[2] = 0;	//B
		}break;		
	}
}
static uint32_t changeL(uint8_t r,uint8_t g,uint8_t b, float light,uint8_t num){
	float h, s, v;
	uint8_t cmax, cmin, cdes;float k=0;
    uint32_t returnColor=0;
	float f, p, q, t;
	float rf, gf, bf;
	int i ;
	k=light/num;
	cmax = r > g ? r : g;
	if (b > cmax)
		cmax = b;
	cmin = r < g ? r : g;
	if (b < cmin)
		cmin = b;
	cdes = cmax - cmin;
	v = cmax / 255.0f;
	s = cmax == 0 ? 0 : cdes / (float) cmax;
	h = 0;
	if (cmax == r && g >= b){
		h = ((g - b) * 60.0f / cdes) + 0;
	}else if (cmax == r && g < b){
		h = ((g - b) * 60.0f / cdes) + 360;
	}else if (cmax == g){
		h = ((b - r) * 60.0f / cdes) + 120;
	}else{
		h = ((r - g) * 60.0f / cdes) + 240;
	}
	//////
	v *= k;
	i = ((int) (h / 60) % 6);
	f = (h / 60) - i;
	p = v * (1 - s);
	q = v * (1 - f * s);
	t = v * (1 - (1 - f) * s);
	switch (i){
		case 0:{
			rf = v;
			gf = t;
			bf = p;
		}break;
		case 1:{
			rf = q;
			gf = v;
			bf = p;
		}break;
		case 2:{
			rf = p;
			gf = v;
			bf = t;
		}break;
		case 3:{
			rf = p;
			gf = q;
			bf = v;
		}break;
		case 4:{
			rf = t;
			gf = p;
			bf = v;
		}break;
		case 5:{
			rf = v;
			gf = p;
			bf = q;
		}break;
		default:
			break;
	}
	r = (uint8_t) (rf * 255.0);
	g = (uint8_t) (gf * 255.0);
	b = (uint8_t) (bf * 255.0);
	returnColor = ((uint32_t) r << 16) | ((uint32_t) g << 8) | b;
	return returnColor;
}
//呼吸功能函数
static void WS2812_Breathing(uint8_t LedType,uint8_t *color, uint16_t len){
    uint8_t i;
	uint16_t buffersize = 0;
    buffersize = (len * 24) +1;       // number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	int memaddr = 0;
	uint16_t result_len=0;
	result_len = len;
    while(result_len){
        /*  green data */
        for(i = 0; i < 8; i++){
            dmaBuffer[memaddr] = ((color[1] << i) & 0x0080) ? TIMING_ONE : TIMING_ZERO;
			memaddr++;
        }
        /*  red data */
        for(i = 0; i < 8; i++){   
            dmaBuffer[memaddr] = ((color[0] << i) & 0x0080) ? TIMING_ONE : TIMING_ZERO;
			memaddr++;
        }

        /*  blue data */
        for(i = 0; i < 8; i++){
            dmaBuffer[memaddr] = ((color[2] << i) & 0x0080) ? TIMING_ONE : TIMING_ZERO;
			memaddr++;
        }
        result_len--;
    }
	if(wsLedSend_func!=NULL){
		wsLedSend_func(LedType,dmaBuffer,buffersize);
	}
}
//顺时针逐个亮流水功能函数
static void WS2812_FluxionPositive(uint8_t LedType,uint8_t *color, uint16_t len,uint16_t dif){
	int i,j;
	uint16_t memaddr2 = 0;
	uint16_t buffersize =0;	
	uint16_t Flowing_num=0;
	uint16_t result_len=0;
	buffersize = (len*24)+1;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	result_len=dif;
	while (result_len){	
		for(i=0; i<8; i++){ // RED
			dmaBuffer[memaddr2] = ((color[1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr2++;
		}
		for(i=0; i<8; i++){ // BLUE
			dmaBuffer[memaddr2] = ((color[0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr2++;
		}
		for(i=0; i<8; i++){ // GREEN data
			dmaBuffer[memaddr2] = ((color[2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr2++;
		}
		result_len--;
	}
	//当前灯灭的位置
	Flowing_num=len-dif;
	for(j=0;j<Flowing_num;j++){  
		for(i=0; i<8; i++){ // GREEN data
			dmaBuffer[memaddr2] = TIMING_ZERO;
			memaddr2++;
		}
		for(i=0; i<8; i++){ // RED
			dmaBuffer[memaddr2] = TIMING_ZERO;
			memaddr2++;
		}
		for(i=0; i<8; i++){ // BLUE
			dmaBuffer[memaddr2] = TIMING_ZERO;
			memaddr2++;
		}
	}
	if(wsLedSend_func!=NULL){
		wsLedSend_func(LedType,dmaBuffer,buffersize);
	}
}

//逆时针逐个亮流水功能函数
static void WS2812_FluxionReverse(uint8_t LedType,uint8_t *color, uint16_t len,uint16_t dif){
	int i,j;
	uint16_t memaddr2 = 0;
	uint16_t buffersize = 0;	
	uint16_t Flowing_num=0;
	uint16_t result_len=0;
	buffersize = (len*24)+1;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	//当前灯灭的位置
	for(j=0;j<dif;j++){  
		for(i=0; i<8; i++){ // GREEN data
			dmaBuffer[memaddr2] = TIMING_ZERO;
			memaddr2++;
		}
		for(i=0; i<8; i++){ // RED
			dmaBuffer[memaddr2] = TIMING_ZERO;
			memaddr2++;
		}
		for(i=0; i<8; i++){ // BLUE
			dmaBuffer[memaddr2] = TIMING_ZERO;
			memaddr2++;
		}
	}
	result_len=len-dif;
	while (result_len){	
		for(i=0; i<8; i++){ // RED
			dmaBuffer[memaddr2] = ((color[1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr2++;
		}
		for(i=0; i<8; i++){ // BLUE
			dmaBuffer[memaddr2] = ((color[0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr2++;
		}
		for(i=0; i<8; i++){ // GREEN data
			dmaBuffer[memaddr2] = ((color[2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr2++;
		}
		result_len--;
	}
	if(wsLedSend_func!=NULL){
		wsLedSend_func(LedType,dmaBuffer,buffersize);
	}
}
//单个灯流水功能函数
static void WS2812_OneLenFluxion(uint8_t LedType,uint8_t *color, uint16_t len,uint16_t NowPoint){
	int i;
	uint16_t memaddr2 = 0;
	uint16_t buffersize = 0;	
	uint16_t Flowing_num=0;
	uint16_t result_len=0;
	buffersize = (len*24)+1;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes

	//result_len=len;
	for(result_len = 0;result_len<len;result_len++){	
		if(result_len == NowPoint){
			for(i=0; i<8; i++){ // RED
				dmaBuffer[memaddr2] = ((color[1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
				memaddr2++;
			}
			for(i=0; i<8; i++){ // BLUE
				dmaBuffer[memaddr2] = ((color[0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
				memaddr2++;
			}
			for(i=0; i<8; i++){ // GREEN data
				dmaBuffer[memaddr2] = ((color[2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
				memaddr2++;
			}		
		}else{//当前灯灭的位置
			for(i=0; i<8; i++){// GREEN data
				dmaBuffer[memaddr2] = TIMING_ZERO;
				memaddr2++;
			}
			for(i=0; i<8; i++){// RED
				dmaBuffer[memaddr2] = TIMING_ZERO;
				memaddr2++;
			}
			for(i=0; i<8; i++){// BLUE
				dmaBuffer[memaddr2] = TIMING_ZERO;
				memaddr2++;
			}
		}	
	}
	if(wsLedSend_func!=NULL){
		wsLedSend_func(LedType,dmaBuffer,buffersize);
	}
}
//顺时针逐个亮流水
static void PositiveWaterfallMode(uint8_t LedType,uint8_t *color,uint16_t LenNum,uint16_t delay_data){ 
	uint16_t i;
	for(i=0;i<LenNum+1;i++){
		WS2812_FluxionPositive(LedType,color,LenNum,i);		
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data-LenNum*2);	
		}				
	}
	WS2812_Breathing(LedType,BlackColor,LenNum);
	if(wsDelay_func!=NULL){
			wsDelay_func(100);	
	}

}
//逆时针逐个灯亮流水
static void ReverseWaterfallMode(uint8_t LedType,uint8_t *color,uint16_t LenNum,uint16_t delay_data){	
	uint16_t i;
	for(i=LenNum;i>=0;i--){
		WS2812_FluxionReverse(LedType,color,LenNum,i);	
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data);
		}				
	}
	WS2812_Breathing(LedType,BlackColor,LenNum);
	if(wsDelay_func!=NULL){
		wsDelay_func(delay_data);
	}	
	
}
//来回逐个灯亮流水
static void ReciprocatingWaterfallMode(uint8_t LedType,uint8_t *color,uint16_t LenNum,uint16_t delay_data){ 
	uint16_t i;
	for(i=0;i<LenNum+1;i++){
		WS2812_FluxionPositive(LedType,color,LenNum,i);	
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data);
		}		
	}
	for(i=LenNum;i>=0;i--){
		WS2812_FluxionReverse(LedType,color,LenNum,i);	
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data);
		}		
	}
	WS2812_Breathing(LedType,BlackColor,LenNum);
	if(wsDelay_func!=NULL){
		wsDelay_func(delay_data);
	}
}
//顺时针单个灯亮流水
static void PositiveWaterfallOneMode(uint8_t LedType,uint8_t *color,uint16_t LenNum,uint16_t delay_data){
	uint16_t i;
	for(i=0;i<LenNum+1;i++){
		WS2812_OneLenFluxion(LedType,color,LenNum,i);
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data);
		}
	}		
}
//逆时针单个灯亮流水
static void ReversePositiveWaterfallOneMode(uint8_t LedType,uint8_t *color,uint16_t LenNum,uint16_t delay_data){
	uint16_t i;
	for(i=LenNum;i>=0;i--){
		WS2812_OneLenFluxion(LedType,color,LenNum,i);
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data);
		}
	}	
}
//来回单个灯亮流水
static void ReciprocatingWaterfallOneMode(uint8_t LedType,uint8_t *color,uint16_t LenNum,uint16_t delay_data){
	uint16_t i;
	for(i=0;i<LenNum;i++){
		WS2812_OneLenFluxion(LedType,color,LenNum,i);
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data);
		}
	}
	for(i=LenNum;i>=0;i--){
		WS2812_OneLenFluxion(LedType,color,LenNum,i);
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data);
		}
	}		
}
//**呼吸用这个
static void BrightnessMode(uint8_t LedType,uint8_t *color,uint16_t LenNum,uint16_t delay_data){   
	uint16_t i;
	uint8_t rgbRed[3] = {0,0,0};
	uint32_t tmp_brightness=0;
	uint8_t rgbRed2[3] = {0,0,0};
	uint32_t tmp_brightness2=0;
	for(i=0;i<50;i++){ 
		tmp_brightness = changeL(color[0] ,color[1] ,color[2], i,50);
		rgbRed[0]= (tmp_brightness & 0x00ff0000)>>16;
		rgbRed[1]= (tmp_brightness & 0x00ff00)>>8;
		rgbRed[2]= tmp_brightness & 0x000000ff;
		WS2812_Breathing(LedType,rgbRed,LenNum);
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data);
		}
	}
	for(i=50;i>0;i--){ 
		tmp_brightness=changeL(color[0] ,color[1] ,color[2], i,50);
		rgbRed[0]=(tmp_brightness & 0x00ff0000)>>16;
		rgbRed[1]=(tmp_brightness & 0x00ff00)>>8;
		rgbRed[2]=tmp_brightness & 0x000000ff;
		WS2812_Breathing(LedType,rgbRed,LenNum);
		if(wsDelay_func!=NULL){
			wsDelay_func(delay_data);
		}	
	}	
}
//**亮灭模式
static void BrightOutMode(uint8_t LedType,uint8_t *color,uint16_t LenNum,uint16_t delay_data){   
	WS2812_Breathing(LedType,color,LenNum);
	if(wsDelay_func!=NULL){
		wsDelay_func(delay_data);
	}
	WS2812_Breathing(LedType,BlackColor,LenNum);
	if(wsDelay_func!=NULL){
		wsDelay_func(delay_data);
	}
}
//开启某个颜色
static void OnLedColorMode(uint8_t LedType,uint8_t *color,uint16_t LenNum){	
	WS2812_Breathing(LedType,color,LenNum);		
}
//关闭显示颜色
static void OffLedColorMode(uint8_t LedType,uint16_t LenNum){
	WS2812_Breathing(LedType,BlackColor,LenNum);			
}
//WS2812功能函数调用
///函数名称：OperatingFun
///输入参数：InputColor->颜色
//1->红 2->绿 3->蓝 4->白 5->橙 6->黄
///输入参数：LenNum->LED灯个数
///输入参数：delay_data->延时时间
///输入参数：MainMode->模式
//1->关闭显示 2->某个颜色常亮 3->某个颜色亮灭 4->某个颜色顺时针逐个亮流水 5->某个颜色逆时针逐个亮流水
//6->某个颜色来回逐个亮流水 7->某个颜色顺时针单个亮流水 8->某个颜色逆时针单个亮流水 
//9->某个颜色来回单个亮流水10->某个颜色呼吸
void OperatingFun(uint8_t ledType,uint16_t InputColor,uint16_t LenNum,uint16_t delay_data,uint16_t MainMode){	
	GetColorPix(ColorBuff,InputColor);
	switch(MainMode){
		case 1: //关闭显示
			OffLedColorMode(ledType,LenNum);
			break;
		case 2://开启某个颜色
			OnLedColorMode(ledType,ColorBuff,LenNum);
			break;
		case 3://亮灭
			BrightOutMode(ledType,ColorBuff,LenNum,delay_data);
			break;
		case 4://顺时针逐个亮流水
			PositiveWaterfallMode(ledType,ColorBuff,LenNum,delay_data);
			break;
		case 5://逆时针逐个亮流水
			ReverseWaterfallMode(ledType,ColorBuff,LenNum,delay_data);
			break;
		case 6://来回逐个亮流水
			ReciprocatingWaterfallMode(ledType,ColorBuff,LenNum,delay_data);
			break;
		case 7://顺时针单个亮流水
			PositiveWaterfallOneMode(ledType,ColorBuff,LenNum,delay_data);
			break;
		case 8://逆时针单个亮流水
			ReversePositiveWaterfallOneMode(ledType,ColorBuff,LenNum,delay_data);
			break;
		case 9://来回单个亮流水
			ReciprocatingWaterfallOneMode(ledType,ColorBuff,LenNum,delay_data);
			break;
		case 10://呼吸
			BrightnessMode(ledType,ColorBuff,LenNum,delay_data);
			break;
	}	
}
	
#ifdef __cplusplus
}
#endif

