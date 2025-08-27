#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

/* Used Flash pages for EEPROM emulation 使用Flash页面进行EEPROM仿真*/
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001)

/* No valid page define    没有有效的页面定义*/
#define NO_VALID_PAGE         ((uint16_t)0x00AB)
/* Page status definitions 页面状态的定义*/
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty 页面是空的*/
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data 页面被标记为接收数据*/
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data     包含有效数据的页面*/

/* Valid pages in read and write defines 有效的读和写页面定义*/
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define 						页面完整的定义*/
#define PAGE_FULL             ((uint8_t)0x80)

uint16_t EE_Init(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

#endif /* __EEPROM_H */




