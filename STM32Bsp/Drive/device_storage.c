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
 
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 

uint8_t STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
	FLASH_Status status = FLASH_COMPLETE;
	uint32_t addrx=0;
	uint32_t endaddr=0;	
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return 0;				//�Ƿ���ַ
	FLASH_Unlock();														//���� 
	FLASH_DataCacheCmd(DISABLE);										//FLASH�����ڼ�,�����ֹ���ݻ���
	addrx=WriteAddr;													//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;										//д��Ľ�����ַ
	if(addrx<0X1FFF0000){												//ֻ�����洢��,����Ҫִ�в�������!!
		while(addrx<endaddr){											//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF){ 					//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)return 1;						//����������
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE){
		while(WriteAddr<endaddr){										//д����
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE){ 	//д������
				return 1;												//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
	return 0;
} 

void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);							//��ȡ4���ֽ�.
		ReadAddr+=4;													//ƫ��4���ֽ�.	
	}
}
void STMFLASH_Erase(uint32_t ErAddr){
	FLASH_Unlock();														//���� 
	FLASH_DataCacheCmd(DISABLE);										//FLASH�����ڼ�,�����ֹ���ݻ���
	FLASH_EraseSector(STMFLASH_GetFlashSector(ErAddr),VoltageRange_3);	//VCC=2.7~3.6V֮��!!
	FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
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
/********************************************���º���lib��ʹ�ò���ɾ��*************************************************/
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