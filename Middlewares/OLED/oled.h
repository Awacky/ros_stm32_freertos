#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx.h"
#include "oledfont.h"

#define OLED_DEV_ADDR 0x78

#define OLED_CMD  0	    //写命令
#define OLED_DATA 0x40	//写数据

void OLED_Init(uint8_t setDma);															//初始化
void OLED_WR_CMD(uint8_t CMD);															//写指令							
void OLED_WR_Dat(uint8_t CMD);															//写数据
void OLED_Set_Y(uint8_t y); 															//行刷新控制
void OLED_Refreash(void);																//DMA刷新函数
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t SIZE,uint8_t Is_Reverse);	//显示单个字符
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t* str,uint8_t SIZE);					//显示字符串
void OLED_Show_Num(uint8_t x, uint8_t y,int Num,uint8_t len,uint8_t SIZE);				//显示数据
void OLED_Show_Num_Reverse(uint8_t x, uint8_t y,int Num,uint8_t len,uint8_t SIZE);		//显示白底数据
void OLED_Clear(void);																	//清屏
void OLED_Draw_Dot(uint8_t x,uint8_t y);												//画点
void OLED_Draw_Line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);						//划线
void OLED_Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r);								//画圆
void OLED_Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);			//画矩形

#endif




