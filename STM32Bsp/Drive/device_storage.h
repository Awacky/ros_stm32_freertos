#ifdef __cplusplus
extern "C" {
#endif
	#ifndef __DEVICE_STORAGE_H__
	#define __DEVICE_STORAGE_H__
	#include "datatypes.h"
	#include "stm32f4xx.h"
	//FLASH��ʼ��ַ
	#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ
	//FLASH ��������ʼ��ַ
	#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
	#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
	#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
	#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
	#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
	#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
	#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
	#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
	#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
	#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
	#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
	#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes  

	uint8_t STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);
	void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);
	void STMFLASH_Erase(uint32_t ErAddr);
	uint8_t* flash_helper_get_sector_address(uint32_t fsector);
	void FLASH_Interface_UnlockClearFlag(void);
	void FLASH_Interface_Lock(void);
	/********************************************���º���lib��ʹ�ò���ɾ��*************************************************/
	void flash_unlock(void);
	uint8_t flash_slib_state_get(void);
	void flash_slib_enable(uint32_t pass_t,uint32_t addrSector1,uint32_t addrSector2,uint32_t addrSector3);
	void flash_sector_erase(uint32_t addr_t);
	void flash_byte_program(uint32_t addr_t,uint8_t Value);
	void flash_lock(void);
	void flash_slib_disable(uint32_t pass_t);	
	/********************************************************************************************************************/
	#endif

#ifdef __cplusplus
}
#endif

