#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

/* Used Flash pages for EEPROM emulation ʹ��Flashҳ�����EEPROM����*/
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001)

/* No valid page define    û����Ч��ҳ�涨��*/
#define NO_VALID_PAGE         ((uint16_t)0x00AB)
/* Page status definitions ҳ��״̬�Ķ���*/
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty ҳ���ǿյ�*/
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data ҳ�汻���Ϊ��������*/
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data     ������Ч���ݵ�ҳ��*/

/* Valid pages in read and write defines ��Ч�Ķ���дҳ�涨��*/
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define 						ҳ�������Ķ���*/
#define PAGE_FULL             ((uint8_t)0x80)

uint16_t EE_Init(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

#endif /* __EEPROM_H */




