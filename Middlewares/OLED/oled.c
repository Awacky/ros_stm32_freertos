#include "oled.h"
#include "devi2c.h"
#include "string.h"

#define Max_Column	128
#define Max_Row		64
#define Reverse    1
#define No_Reverse 0


static uint8_t IsDma=0;
uint8_t OLED_Init_CMD[] = {
	0xAE,0x00,0x10,0x40,0xB0,0x81,0xFF,0xA1,0xA6,0xA8,\
	0x3F,0xC8,0xD3,0x00,0xD5,0x80,0xD8,0x05,0xD9,0xF1,\
	0xDA,0x12,0xDB,0x30,0x8D,0x14,0xAF,0x20,0x00};
uint8_t OLED_Frame_Buffer[8][128]={0};
uint8_t Is_Init = 0x10; 		// 高四位 是 是否进入刷新

/**
  * @brief  OLED初始化
  * @param  setDma 启用DMA
  * @retval 无
  */
void OLED_Init(uint8_t setDma)
{ 						
	uint8_t i;
	Is_Init = 0x10;
	IsDma = setDma;
	DevHwI2C_Init();		// 初始化 IIC PORT
	if(setDma){
		DevDmaI2C_Init();	// 初始化 DMA IIC
	}
	for(i=0;i<29;i++) {		// 顺序写入这些命令完成初始化函数
		DevDmaI2c_WriteReg(&OLED_Init_CMD[i], 0x78,0x00,1); // 初始化OLED
	}
	Is_Init = 0x00;
}
/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(unsigned char Command)//写命令
{
	DevHwI2C_WriteByte(OLED_DEV_ADDR,0x00, Command);
}
 
/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
*/
void OLED_WriteData(unsigned char Data)//写数据
{
	DevHwI2C_WriteByte(OLED_DEV_ADDR,0x40, Data);
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t X, uint8_t Y)
{
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置低4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置高4位
}

/**
  * @brief  OLED部分清屏
  * @param  Line 行位置，范围：1~4
  * @param  start 列开始位置，范围：1~16
  * @param  end 列开始位置，范围：1~16
  * @retval 无
  */
void OLED_Clear_Part(uint8_t Line, uint8_t start, uint8_t end)
{  
	uint8_t i,Column;
	for(Column = start; Column <= end; Column++){
		OLED_SetCursor( (Column - 1) * 8,(Line - 1) * 2);		//设置光标位置在上半部分
		for (i = 0; i < 8; i++){
			OLED_WriteData(0x00);								//显示上半部分内容
		}
		OLED_SetCursor( (Column - 1) * 8,(Line - 1) * 2 + 1);	//设置光标位置在下半部分
		for (i = 0; i < 8; i++){
			OLED_WriteData(0x00);								//显示下半部分内容
		}
	}
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--){
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length,uint8_t SIZE)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++){
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10){
			OLED_ShowChar(Line, Column + i, SingleNumber + '0',SIZE,0);
		}else{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A',SIZE,0);
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length,uint8_t SIZE)
{
	uint8_t i;
	for (i = 0; i < Length; i++){
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0',SIZE,0);
	}
}
/**
  * @brief  OLED画矩形的函数， 本质就是画4条线
  * @param  x1 列起点
  * @param  y1 行起点
  * @param  x2 列终点
  * @param  y2 行终点
  * @retval 无
  */
void OLED_Draw_Rectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	OLED_Draw_Line(x1,y1,x2,y1);
	OLED_Draw_Line(x1,y1,x1,y2);
	OLED_Draw_Line(x1,y2,x2,y2);
	OLED_Draw_Line(x2,y1,x2,y2);
}
/**
  * @brief  OLED画圆的函数
  * @param  x0 列起点
  * @param  y0 行起点
  * @param  r  园半径
  * @retval 无
  */
void OLED_Draw_Circle(u16 x0,u16 y0,uint8_t r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //判断下个点位置的标志
	while(a<=b){
		OLED_Draw_Dot(x0+a,y0-b);             //5
 		OLED_Draw_Dot(x0+b,y0-a);             //0           
		OLED_Draw_Dot(x0+b,y0+a);             //4               
		OLED_Draw_Dot(x0+a,y0+b);             //6 
		OLED_Draw_Dot(x0-a,y0+b);             //1       
 		OLED_Draw_Dot(x0-b,y0+a);             
		OLED_Draw_Dot(x0-a,y0-b);             //2             
		OLED_Draw_Dot(x0-b,y0-a);             //7     	         
		a++;
		//使用Bresenham算法画圆     
		if(di<0)di +=4*a+6;	  
		else{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
}

/**
  * @brief  OLED画线的函数
  * @param  x1 列起点
  * @param  y1 行起点
  * @param  x2 列终点
  * @param  y2 行终点
  * @retval 无
  */
void OLED_Draw_Line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{ 
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; 							//计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; 					//设置单步方向 
	else if(delta_x==0)incx=0;				//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;				//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; 	//选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )				//画线输出 
	{  
		OLED_Draw_Dot(uRow,uCol);			//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) { 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) { 
			yerr-=distance; 
			uCol+=incy; 
		}
	}		
}
/**
  * @brief  OLED画点函数 ，自己写的 本质就是将点的坐标转换成数组里面 指定数据的指定位
  * @param  x 列
  * @param  y 行
  * @retval 无
  */
void OLED_Draw_Dot(uint8_t x,uint8_t y)
{ 
	OLED_Frame_Buffer[y/8][x] |= 1<<(y%8);
}
/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{ 
	uint8_t j,i;
	if(IsDma){
		for(j=0;j<128;j++){
			OLED_Frame_Buffer[0][j] =0;
			OLED_Frame_Buffer[1][j] =0;
			OLED_Frame_Buffer[2][j] =0;
			OLED_Frame_Buffer[3][j] =0;
			OLED_Frame_Buffer[4][j] =0;
			OLED_Frame_Buffer[5][j] =0;
			OLED_Frame_Buffer[6][j] =0;
			OLED_Frame_Buffer[7][j] =0;
		}
	}else{
		for (j = 0; j < 8; j++){
			OLED_SetCursor(0,j);
			for(i = 0; i < 128; i++){
				OLED_WriteData(0x00);
			}
		}
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  y 起始行位置，范围：1~4
  * @param  x 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_Show_Num(uint8_t x, uint8_t y,int Num,uint8_t len,uint8_t SIZE)
{ 
	if(IsDma){
		if(SIZE <=8){
			if(Num<0){
				OLED_ShowChar(x,y,'-',8,No_Reverse);
				Num *=-1;// 相较于官方的代码 添加了显示负数的支持
			}
			while (len){		
				OLED_ShowChar(x+6*len,y,(Num%10)+'0',8,No_Reverse);
				Num/=10;// 此处直接反向更新数据相比于官方的计算方案
				len--;  // 减少了一定的运算量,缺点是第一位如果是0 也会显示
			}
		}else{
			if(Num<0){
				OLED_ShowChar(x,y,'-',16,No_Reverse);
				Num *=-1;
			}
			while (len){		
				OLED_ShowChar(x+8*len,y,(Num%10)+'0',16,No_Reverse);
				Num/=10;
				len--;
			}	
		}
	}else{
		uint8_t i;
		uint32_t Number1;
		if (Num >= 0){
			OLED_ShowChar(x, y, '+',SIZE,0);
			Number1 = Num;
		}else{
			OLED_ShowChar(x, y, '-',SIZE,0);
			Number1 = -Num;
		}
		for (i = 0; i < len; i++){
			OLED_ShowChar(x, y + i + 1, Number1 / OLED_Pow(10, len - i - 1) % 10 + '0',SIZE,0);
		}
	}
}
/**
  * @brief  OLED显示反白显示函数， 相比较于上一个函数，写入的数字按位取反了
  * @param  y 行位置，范围：1~4
  * @param  x 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowNum_Reverse(uint8_t x, uint8_t y,int Num,uint8_t len,uint8_t SIZE)
{ 
	if(SIZE <=8){
		if(Num<0){
			OLED_ShowChar(x,y,'-',8,Reverse);
			Num *=-1;
		}
		while (len){		
			OLED_ShowChar(x+6*len,y,(Num%10)+'0',8,Reverse);
			Num/=10;
			len--;
		}
	}else{
		if(Num<0){
			OLED_ShowChar(x,y,'-',16,0);
			Num *=-1;
		}
		while (len){		
			OLED_ShowChar(x+8*len,y,(Num%10)+'0',16,Reverse);
			Num/=10;
			len--;
		}	
	}
}
/**
  * @brief  OLED显示一个字符串
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t* str,uint8_t SIZE)
{ // 字符串显示函数，依次写入每一个字符
	uint8_t i=0;
	switch(SIZE){
		case 1:
		case 8:{
			while (str[i]!='\0'){		
				OLED_ShowChar(x,y,str[i],8,No_Reverse);
				x+=6;
				i++;
			}
			if(x<128){
				OLED_ShowChar(x,y,' ',8,No_Reverse);
				x+=6;
			}
		}break;
		case 2:
		case 16:{
			while (str[i]!='\0'){		
				OLED_ShowChar(x,y,str[i],16,No_Reverse);
				x+=8;
				i++;			
			}
		}break;
	}
}
/**
  * @brief  OLED显示一个字符
  * @param  y 行位置，范围: 0~7 0~3
  * @param  x 列位置，范围：1~32
  * @param  chr 要显示的一个字符，范围：ASCII可见字符
  * @param  Is_Reverse 是否白底反向
  * @retval 无
  */
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t SIZE,uint8_t Is_Reverse)
{
	//字符显示函数，这里是依次的将字符的字库信息转存到显示Buffer里面
	uint8_t c,i;uint8_t yVal1,yVal2;	
	c=chr-' ';//得到偏移后的值		
	if(IsDma){
		switch(SIZE){
			case 1:
			case 8:{	//6*8	
				y=y+1;
				if(y>7)y=0;				
				if(Is_Reverse==0){
					for(i=0;i<6;i++){
						OLED_Frame_Buffer[y][x+i] = F6x8[c][i] ;
					}
				}else{
					for(i=0;i<6;i++){
						OLED_Frame_Buffer[y][x+i] = ~F6x8[c][i] ;
					}
				}
			}break;
			case 2:
			case 16:{// 8*16			
				if(y>3){y=3;}
				if(y==3){
					yVal1 = 7;yVal2=0;
				}else{
					yVal1 = 2*y+1;yVal2 = 2*y+2;
				}
				if(Is_Reverse == 0){
					for(i=0;i<8;i++){
						OLED_Frame_Buffer[yVal1][x+i] = F8X16[c*16+i  ];
						OLED_Frame_Buffer[yVal2][x+i] = F8X16[c*16+i+8];						
					}
				}else{
					for(i=0;i<8;i++){
						OLED_Frame_Buffer[yVal1][x+i] = ~F8X16[c*16+i  ];
						OLED_Frame_Buffer[yVal2][x+i] = ~F8X16[c*16+i+8];
					}
				}
			}break;
		}
	}else{
		switch(SIZE){
			case 1:
			case 8:{
				OLED_SetCursor(x,y);									//设置光标位置在上半部分
				for (i = 0; i < 6; i++){
					OLED_WriteData(F6x8[c][i]);					  
				}
			}break;
			case 2:
			case 16:{
				OLED_SetCursor(x,y* 2);									//设置光标位置在上半部分
				for (i = 0; i < 8; i++){
					OLED_WriteData(F8X16[c*16+i  ]);					//显示上半部分内容
				}
				OLED_SetCursor(x,y* 2 + 1);								//设置光标位置在下半部分
				for (i = 0; i < 8; i++){
					OLED_WriteData(F8X16[c*16+i+8]);					//显示下半部分内容
				}
			}break;
		}
	}	
}

void OLED_Refreash(void)
{	// OLED刷新函数, 手动的将 1024个byte传递给GDDRAM  DMA中断会自动调用，不需要自己调用
	DevDmaI2c_WriteReg(OLED_Frame_Buffer[0], OLED_DEV_ADDR,0x40,1024);
}
void OLED_WR_CMD(uint8_t CMD)
{ // OLED 写指令 的操作方式 : 依次发送 0x78 0x40 CMD
	DevDmaI2c_WriteReg(&CMD, OLED_DEV_ADDR,0x00,1);
}
void OLED_WR_Dat(uint8_t CMD)
	{ // OLED 写数据的方式 : 依次发送 0x78 0x00 dat1 dat2 .... datn
	DevDmaI2c_WriteReg(&CMD, OLED_DEV_ADDR,0x40,1);
}
void OLED_Set_Y(uint8_t y) // 行刷新控制
{ // 单行刷新是的程序  用不到了
	OLED_WR_CMD(0xb0+y);
}



