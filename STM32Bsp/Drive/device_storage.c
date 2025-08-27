#ifdef __cplusplus
extern "C" {
#endif
#include "device_storage.h"
#include "hw_config.h"

#define FLASH_SECTORS							12

static Hardware_Struct hStr_Hard;
static uint32_t STMFLASH_ReadWord(uint32_t faddr);
static uint16_t STMFLASH_GetFlashSector(uint32_t addr); 
// Private constants
static const uint32_t flash_addr[FLASH_SECTORS] = {
		ADDR_FLASH_SECTOR_0,
		ADDR_FLASH_SECTOR_1,
		ADDR_FLASH_SECTOR_2,
		ADDR_FLASH_SECTOR_3,
		ADDR_FLASH_SECTOR_4,
		ADDR_FLASH_SECTOR_5,
		ADDR_FLASH_SECTOR_6,
		ADDR_FLASH_SECTOR_7,
		ADDR_FLASH_SECTOR_8,
		ADDR_FLASH_SECTOR_9,
		ADDR_FLASH_SECTOR_10,
		ADDR_FLASH_SECTOR_11
};
static const uint16_t flash_sector[FLASH_SECTORS] = {
		FLASH_Sector_0,
		FLASH_Sector_1,
		FLASH_Sector_2,
		FLASH_Sector_3,
		FLASH_Sector_4,
		FLASH_Sector_5,
		FLASH_Sector_6,
		FLASH_Sector_7,
		FLASH_Sector_8,
		FLASH_Sector_9,
		FLASH_Sector_10,
		FLASH_Sector_11
};	
static uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(uint32_t*)faddr; 
} 
static uint16_t STMFLASH_GetFlashSector(uint32_t addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
 
//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 

uint8_t STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
	FLASH_Status status = FLASH_COMPLETE;
	uint32_t addrx=0;
	uint32_t endaddr=0;	
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return 0;				//非法地址
	FLASH_Unlock();														//解锁 
	FLASH_DataCacheCmd(DISABLE);										//FLASH擦除期间,必须禁止数据缓存
	addrx=WriteAddr;													//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;										//写入的结束地址
	if(addrx<0X1FFF0000){												//只有主存储区,才需要执行擦除操作!!
		while(addrx<endaddr){											//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF){ 					//有非0XFFFFFFFF的地方,要擦除这个扇区
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)return 1;						//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE){
		while(WriteAddr<endaddr){										//写数据
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE){ 	//写入数据
				return 1;												//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
	return 0;
} 

void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);							//读取4个字节.
		ReadAddr+=4;													//偏移4个字节.	
	}
}
void STMFLASH_Erase(uint32_t ErAddr){
	FLASH_Unlock();														//解锁 
	FLASH_DataCacheCmd(DISABLE);										//FLASH擦除期间,必须禁止数据缓存
	FLASH_EraseSector(STMFLASH_GetFlashSector(ErAddr),VoltageRange_3);	//VCC=2.7~3.6V之间!!
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
}
uint8_t* flash_helper_get_sector_address(uint32_t fsector) {
	uint8_t *res = 0;
	for (int i = 0;i < FLASH_SECTORS;i++) {
		if (flash_sector[i] == fsector) {
			res = (uint8_t *)flash_addr[i];
			break;
		}
	}

	return res;
}
void FLASH_Interface_UnlockClearFlag(void){
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}
void FLASH_Interface_Lock(void){
	FLASH_Lock();
}
/********************************************以下函数lib会使用不可删除*************************************************/
void flash_unlock(void)
{
	
}
uint8_t flash_slib_state_get(void){
	return 1;
}
void flash_slib_enable(uint32_t pass_t,uint32_t addrSector1,uint32_t addrSector2,uint32_t addrSector3){
	
}
void flash_sector_erase(uint32_t addr_t){
	
}
void flash_byte_program(uint32_t addr_t,uint8_t Value){
	
}
void flash_lock(void){
	
}
void flash_slib_disable(uint32_t pass_t){
	
}
/********************************************************************************************************************/
#ifdef __cplusplus
}
#endif