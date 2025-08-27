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
uint8_t Is_Init = 0x10; 		// ����λ �� �Ƿ����ˢ��

/**
  * @brief  OLED��ʼ��
  * @param  setDma ����DMA
  * @retval ��
  */
void OLED_Init(uint8_t setDma)
{ 						
	uint8_t i;
	Is_Init = 0x10;
	IsDma = setDma;
	DevHwI2C_Init();		// ��ʼ�� IIC PORT
	if(setDma){
		DevDmaI2C_Init();	// ��ʼ�� DMA IIC
	}
	for(i=0;i<29;i++) {		// ˳��д����Щ������ɳ�ʼ������
		DevDmaI2c_WriteReg(&OLED_Init_CMD[i], 0x78,0x00,1); // ��ʼ��OLED
	}
	Is_Init = 0x00;
}
/**
  * @brief  OLEDд����
  * @param  Command Ҫд�������
  * @retval ��
  */
void OLED_WriteCommand(unsigned char Command)//д����
{
	DevHwI2C_WriteByte(OLED_DEV_ADDR,0x00, Command);
}
 
/**
  * @brief  OLEDд����
  * @param  Data Ҫд�������
  * @retval ��
*/
void OLED_WriteData(unsigned char Data)//д����
{
	DevHwI2C_WriteByte(OLED_DEV_ADDR,0x40, Data);
}

/**
  * @brief  OLED���ù��λ��
  * @param  Y �����Ͻ�Ϊԭ�㣬���·�������꣬��Χ��0~7
  * @param  X �����Ͻ�Ϊԭ�㣬���ҷ�������꣬��Χ��0~127
  * @retval ��
  */
void OLED_SetCursor(uint8_t X, uint8_t Y)
{
	OLED_WriteCommand(0xB0 | Y);					//����Yλ��
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//����Xλ�õ�4λ
	OLED_WriteCommand(0x00 | (X & 0x0F));			//����Xλ�ø�4λ
}

/**
  * @brief  OLED��������
  * @param  Line ��λ�ã���Χ��1~4
  * @param  start �п�ʼλ�ã���Χ��1~16
  * @param  end �п�ʼλ�ã���Χ��1~16
  * @retval ��
  */
void OLED_Clear_Part(uint8_t Line, uint8_t start, uint8_t end)
{  
	uint8_t i,Column;
	for(Column = start; Column <= end; Column++){
		OLED_SetCursor( (Column - 1) * 8,(Line - 1) * 2);		//���ù��λ�����ϰ벿��
		for (i = 0; i < 8; i++){
			OLED_WriteData(0x00);								//��ʾ�ϰ벿������
		}
		OLED_SetCursor( (Column - 1) * 8,(Line - 1) * 2 + 1);	//���ù��λ�����°벿��
		for (i = 0; i < 8; i++){
			OLED_WriteData(0x00);								//��ʾ�°벿������
		}
	}
}

/**
  * @brief  OLED�η�����
  * @retval ����ֵ����X��Y�η�
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
  * @brief  OLED��ʾ���֣�ʮ�����ƣ�������
  * @param  Line ��ʼ��λ�ã���Χ��1~4
  * @param  Column ��ʼ��λ�ã���Χ��1~16
  * @param  Number Ҫ��ʾ�����֣���Χ��0~0xFFFFFFFF
  * @param  Length Ҫ��ʾ���ֵĳ��ȣ���Χ��1~8
  * @retval ��
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
  * @brief  OLED��ʾ���֣������ƣ�������
  * @param  Line ��ʼ��λ�ã���Χ��1~4
  * @param  Column ��ʼ��λ�ã���Χ��1~16
  * @param  Number Ҫ��ʾ�����֣���Χ��0~1111 1111 1111 1111
  * @param  Length Ҫ��ʾ���ֵĳ��ȣ���Χ��1~16
  * @retval ��
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length,uint8_t SIZE)
{
	uint8_t i;
	for (i = 0; i < Length; i++){
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0',SIZE,0);
	}
}
/**
  * @brief  OLED�����εĺ����� ���ʾ��ǻ�4����
  * @param  x1 �����
  * @param  y1 �����
  * @param  x2 ���յ�
  * @param  y2 ���յ�
  * @retval ��
  */
void OLED_Draw_Rectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	OLED_Draw_Line(x1,y1,x2,y1);
	OLED_Draw_Line(x1,y1,x1,y2);
	OLED_Draw_Line(x1,y2,x2,y2);
	OLED_Draw_Line(x2,y1,x2,y2);
}
/**
  * @brief  OLED��Բ�ĺ���
  * @param  x0 �����
  * @param  y0 �����
  * @param  r  ԰�뾶
  * @retval ��
  */
void OLED_Draw_Circle(u16 x0,u16 y0,uint8_t r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //�ж��¸���λ�õı�־
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
		//ʹ��Bresenham�㷨��Բ     
		if(di<0)di +=4*a+6;	  
		else{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
}

/**
  * @brief  OLED���ߵĺ���
  * @param  x1 �����
  * @param  y1 �����
  * @param  x2 ���յ�
  * @param  y2 ���յ�
  * @retval ��
  */
void OLED_Draw_Line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{ 
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; 							//������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; 					//���õ������� 
	else if(delta_x==0)incx=0;				//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;				//ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; 	//ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )				//������� 
	{  
		OLED_Draw_Dot(uRow,uCol);			//���� 
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
  * @brief  OLED���㺯�� ���Լ�д�� ���ʾ��ǽ��������ת������������ ָ�����ݵ�ָ��λ
  * @param  x ��
  * @param  y ��
  * @retval ��
  */
void OLED_Draw_Dot(uint8_t x,uint8_t y)
{ 
	OLED_Frame_Buffer[y/8][x] |= 1<<(y%8);
}
/**
  * @brief  OLED����
  * @param  ��
  * @retval ��
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
  * @brief  OLED��ʾ���֣�ʮ���ƣ�����������
  * @param  y ��ʼ��λ�ã���Χ��1~4
  * @param  x ��ʼ��λ�ã���Χ��1~16
  * @param  Number Ҫ��ʾ�����֣���Χ��-2147483648~2147483647
  * @param  Length Ҫ��ʾ���ֵĳ��ȣ���Χ��1~10
  * @retval ��
  */
void OLED_Show_Num(uint8_t x, uint8_t y,int Num,uint8_t len,uint8_t SIZE)
{ 
	if(IsDma){
		if(SIZE <=8){
			if(Num<0){
				OLED_ShowChar(x,y,'-',8,No_Reverse);
				Num *=-1;// ����ڹٷ��Ĵ��� �������ʾ������֧��
			}
			while (len){		
				OLED_ShowChar(x+6*len,y,(Num%10)+'0',8,No_Reverse);
				Num/=10;// �˴�ֱ�ӷ��������������ڹٷ��ļ��㷽��
				len--;  // ������һ����������,ȱ���ǵ�һλ�����0 Ҳ����ʾ
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
  * @brief  OLED��ʾ������ʾ������ ��Ƚ�����һ��������д������ְ�λȡ����
  * @param  y ��λ�ã���Χ��1~4
  * @param  x ��λ�ã���Χ��1~16
  * @param  Char Ҫ��ʾ��һ���ַ�����Χ��ASCII�ɼ��ַ�
  * @retval ��
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
  * @brief  OLED��ʾһ���ַ���
  * @param  Line ��λ�ã���Χ��1~4
  * @param  Column ��λ�ã���Χ��1~16
  * @param  Char Ҫ��ʾ��һ���ַ�����Χ��ASCII�ɼ��ַ�
  * @retval ��
  */
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t* str,uint8_t SIZE)
{ // �ַ�����ʾ����������д��ÿһ���ַ�
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
  * @brief  OLED��ʾһ���ַ�
  * @param  y ��λ�ã���Χ: 0~7 0~3
  * @param  x ��λ�ã���Χ��1~32
  * @param  chr Ҫ��ʾ��һ���ַ�����Χ��ASCII�ɼ��ַ�
  * @param  Is_Reverse �Ƿ�׵׷���
  * @retval ��
  */
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t SIZE,uint8_t Is_Reverse)
{
	//�ַ���ʾ���������������εĽ��ַ����ֿ���Ϣת�浽��ʾBuffer����
	uint8_t c,i;uint8_t yVal1,yVal2;	
	c=chr-' ';//�õ�ƫ�ƺ��ֵ		
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
				OLED_SetCursor(x,y);									//���ù��λ�����ϰ벿��
				for (i = 0; i < 6; i++){
					OLED_WriteData(F6x8[c][i]);					  
				}
			}break;
			case 2:
			case 16:{
				OLED_SetCursor(x,y* 2);									//���ù��λ�����ϰ벿��
				for (i = 0; i < 8; i++){
					OLED_WriteData(F8X16[c*16+i  ]);					//��ʾ�ϰ벿������
				}
				OLED_SetCursor(x,y* 2 + 1);								//���ù��λ�����°벿��
				for (i = 0; i < 8; i++){
					OLED_WriteData(F8X16[c*16+i+8]);					//��ʾ�°벿������
				}
			}break;
		}
	}	
}

void OLED_Refreash(void)
{	// OLEDˢ�º���, �ֶ��Ľ� 1024��byte���ݸ�GDDRAM  DMA�жϻ��Զ����ã�����Ҫ�Լ�����
	DevDmaI2c_WriteReg(OLED_Frame_Buffer[0], OLED_DEV_ADDR,0x40,1024);
}
void OLED_WR_CMD(uint8_t CMD)
{ // OLED дָ�� �Ĳ�����ʽ : ���η��� 0x78 0x40 CMD
	DevDmaI2c_WriteReg(&CMD, OLED_DEV_ADDR,0x00,1);
}
void OLED_WR_Dat(uint8_t CMD)
	{ // OLED д���ݵķ�ʽ : ���η��� 0x78 0x00 dat1 dat2 .... datn
	DevDmaI2c_WriteReg(&CMD, OLED_DEV_ADDR,0x40,1);
}
void OLED_Set_Y(uint8_t y) // ��ˢ�¿���
{ // ����ˢ���ǵĳ���  �ò�����
	OLED_WR_CMD(0xb0+y);
}



