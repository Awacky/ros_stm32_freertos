#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx.h"
#include "oledfont.h"

#define OLED_DEV_ADDR 0x78

#define OLED_CMD  0	    //д����
#define OLED_DATA 0x40	//д����

void OLED_Init(uint8_t setDma);															//��ʼ��
void OLED_WR_CMD(uint8_t CMD);															//дָ��							
void OLED_WR_Dat(uint8_t CMD);															//д����
void OLED_Set_Y(uint8_t y); 															//��ˢ�¿���
void OLED_Refreash(void);																//DMAˢ�º���
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t SIZE,uint8_t Is_Reverse);	//��ʾ�����ַ�
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t* str,uint8_t SIZE);					//��ʾ�ַ���
void OLED_Show_Num(uint8_t x, uint8_t y,int Num,uint8_t len,uint8_t SIZE);				//��ʾ����
void OLED_Show_Num_Reverse(uint8_t x, uint8_t y,int Num,uint8_t len,uint8_t SIZE);		//��ʾ�׵�����
void OLED_Clear(void);																	//����
void OLED_Draw_Dot(uint8_t x,uint8_t y);												//����
void OLED_Draw_Line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);						//����
void OLED_Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r);								//��Բ
void OLED_Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);			//������

#endif




