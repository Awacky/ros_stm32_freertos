#ifdef __cplusplus
extern "C" {
#endif
	
#ifndef __USART6_H
#define __USART6_H
#include "stdio.h"	
#include "stm32f4xx.h"

#define USART6_REC_LEN  			200  	//�����������ֽ��� 200
#define USART6_RECEIVE_BUF_LENGTH 896

extern char  USART6_RX_BUF[USART6_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART6_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��

extern u16 usart6_send_buf_length;
extern u8 usart6_send_buf[];
extern u16 usart6_rx_buf_length;
extern u8 usart6_rx_buf[];
extern u8 usart6_rx_irq_updata_user_reset_status;
extern u8 usart6_tx_irq_updata_user_reset_status;

void USART6_Init(u8 baudrate_index);
void USART6_DMA_Init(void);
void USART6_DataFrame_Send (unsigned char *send_buf,int length);
void u3_printf(char* fmt,...)  ;
#endif
#ifdef __cplusplus
}
#endif


