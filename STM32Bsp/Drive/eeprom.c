
#include "eeprom.h"
#include "device_storage.h"

uint16_t DataVar = 0;

/* Virtual address defined by the user: 0xFFFF value is prohibited 禁止设置用户定义的虚拟地址:0xFFFF*/
extern uint16_t VirtAddVarTab[NB_OF_VAR];

static FLASH_Status EE_Format(void);
static uint16_t EE_FindValidPage(uint8_t Operation);
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data);
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data);
static uint16_t EE_EraseSectorIfNotEmpty(uint32_t FLASH_Sector, uint8_t VoltageRange);

/**
 * @brief  Restore the pages to a known good state in case of page's status
 *   corruption after a power loss. 如果断电后页面状态损坏，则将页面恢复到已知的良好状态
 * @param  None.
 * @retval - Flash error code: on write Flash error Flash错误码:on write Flash错误
 *         - FLASH_COMPLETE: on success 成功
 */
uint16_t EE_Init(void){
	uint16_t PageStatus0 = 6, PageStatus1 = 6;
	uint16_t VarIdx = 0;
	uint16_t EepromStatus = 0, ReadStatus = 0;
	int16_t x = -1;
	uint16_t  FlashStatus;
	/* Get Page0 status */
	PageStatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);
	/* Get Page1 status */
	PageStatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);
	/* Check for invalid header states and repair if necessary */
	switch (PageStatus0){
		case ERASED:{									//0xFFFF
			if (PageStatus1 == VALID_PAGE){ 			/* Page0 erased, Page1 valid  0x0000*/
				/* Erase Page0 */
				FlashStatus = EE_EraseSectorIfNotEmpty(PAGE0_ID,VOLTAGE_RANGE);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE){
					return FlashStatus;
				}
			}else if (PageStatus1 == RECEIVE_DATA){ 	/* Page0 erased, Page1 receive 0xEEEE*/
				/* Erase Page0 */
				FlashStatus = EE_EraseSectorIfNotEmpty(PAGE0_ID, VOLTAGE_RANGE);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE){
					return FlashStatus;
				}
				/* Mark Page1 as valid */
				FlashStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
				/* If program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE){
					return FlashStatus;
				}
			} else {/* First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM */
					/* Erase both Page0 and Page1 and set Page0 as valid page */
				FlashStatus = EE_Format();
				/* If erase/program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE){
					return FlashStatus;
				}
			}
		}break;

		case RECEIVE_DATA:{								//0xEEEE
			if (PageStatus1 == VALID_PAGE){ /* Page0 receive, Page1 valid */
				/* Transfer data from Page1 to Page0 */
				for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++)
				{
					if (( *(__IO uint16_t*)(PAGE0_BASE_ADDRESS + 6)) == VirtAddVarTab[VarIdx])
					{
						x = VarIdx;
					}
					if (VarIdx != x)
					{
						/* Read the last variables' updates */
						ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
						/* In case variable corresponding to the virtual address was found */
						if (ReadStatus != 0x1)
						{
							/* Transfer the variable to the Page0 */
							EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
							/* If program operation was failed, a Flash error code is returned */
							if (EepromStatus != FLASH_COMPLETE)
							{
								return EepromStatus;
							}
						}
					}
				}
				/* Mark Page0 as valid */
				FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
				/* If program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
				/* Erase Page1 */
				FlashStatus = EE_EraseSectorIfNotEmpty(PAGE1_ID, VOLTAGE_RANGE);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			else if (PageStatus1 == ERASED) /* Page0 receive, Page1 erased */
			{
				/* Erase Page1 */
				FlashStatus = EE_EraseSectorIfNotEmpty(PAGE1_ID, VOLTAGE_RANGE);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
				/* Mark Page0 as valid */
				FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
				/* If program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			else /* Invalid state -> format eeprom */
			{
				/* Erase both Page0 and Page1 and set Page0 as valid page */
				FlashStatus = EE_Format();
				/* If erase/program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
		}break;
		case VALID_PAGE:{								//0x0000
			if (PageStatus1 == VALID_PAGE) /* Invalid state -> format eeprom */
			{
				/* Erase both Page0 and Page1 and set Page0 as valid page */
				FlashStatus = EE_Format();
				/* If erase/program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			else if (PageStatus1 == ERASED) /* Page0 valid, Page1 erased */
			{
				/* Erase Page1 */
				FlashStatus = EE_EraseSectorIfNotEmpty(PAGE1_ID, VOLTAGE_RANGE);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			else /* Page0 valid, Page1 receive */
			{
				/* Transfer data from Page0 to Page1 */
				for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++)
				{
					if ((*(__IO uint16_t*)(PAGE1_BASE_ADDRESS + 6)) == VirtAddVarTab[VarIdx])
					{
						x = VarIdx;
					}
					if (VarIdx != x)
					{
						/* Read the last variables' updates */
						ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
						/* In case variable corresponding to the virtual address was found */
						if (ReadStatus != 0x1)
						{
							/* Transfer the variable to the Page1 */
							EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
							/* If program operation was failed, a Flash error code is returned */
							if (EepromStatus != FLASH_COMPLETE)
							{
								return EepromStatus;
							}
						}
					}
				}
				/* Mark Page1 as valid */
				FlashStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
				/* If program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
				/* Erase Page0 */
				FlashStatus = EE_EraseSectorIfNotEmpty(PAGE0_ID, VOLTAGE_RANGE);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
		}break;

		default:  /* Any other state -> format eeprom 任何其他状态->格式eeprom */
			/* Erase both Page0 and Page1 and set Page0 as valid page 删除Page0和Page1，设置Page0为有效页面*/
			FlashStatus = EE_Format();
			/* If erase/program operation was failed, a Flash error code is returned 如果擦除/程序操作失败，则返回Flash错误码*/
			if (FlashStatus != FLASH_COMPLETE)
			{
				return FlashStatus;
			}
			break;
	}
	return FLASH_COMPLETE;
}

/**
 * @brief  Returns the last stored variable data, if found, which correspond to
 *   the passed virtual address										如果找到，返回最后存储的变量数据，它对应于传递的虚拟地址
 * @param  VirtAddress: Variable virtual address
 * @param  Data: Global variable contains the read variable value	全局变量包含读取变量的值
 * @retval Success or error status:
 *           - 0: if variable was found								如果找到变量
 *           - 1: if the variable was not found						如果没有找到变量
 *           - NO_VALID_PAGE: if no valid page was found.			如果没有找到有效的页面。
 */
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data)
{
	uint16_t ValidPage = PAGE0;
	uint16_t AddressValue = 0x5555, ReadStatus = 1;
	uint32_t Address = EEPROM_START_ADDRESS, PageStartAddress = EEPROM_START_ADDRESS;
	/* Get active Page for read operation */
	ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);
	/* Check if there is no valid page */
	if (ValidPage == NO_VALID_PAGE){
		return  NO_VALID_PAGE;
	}
	/* Get the valid Page start Address */
	PageStartAddress = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));
	/* Get the valid Page end Address */
	Address = (uint32_t)((EEPROM_START_ADDRESS - 2) + (uint32_t)((1 + ValidPage) * PAGE_SIZE));
	/* Check each active page address starting from end */
	while (Address > (PageStartAddress + 2)){
		/* Get the current location content to be compared with virtual address */
		AddressValue = (*(__IO uint16_t*)Address);
		/* Compare the read address with the virtual address */
		if (AddressValue == VirtAddress){
			/* Get content of Address-2 which is variable value */
			*Data = (*(__IO uint16_t*)(Address - 2));
			/* In case variable value is read, reset ReadStatus flag */
			ReadStatus = 0;
			break;
		}else{
			/* Next address location */
			Address = Address - 4;
		}
	}
	/* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
	return ReadStatus;
}
/**
 * @brief  Writes/upadtes variable data in EEPROM.
 * @param  VirtAddress: Variable virtual address
 * @param  Data: 16 bit data to be written
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data){
	uint16_t data_old = 0;
	if (EE_ReadVariable(VirtAddress, &data_old) == 0) {
		if (data_old == Data){
			return FLASH_COMPLETE;
		}
	}
	uint16_t Status = 0;
	/* Return error if MCU VDD is below 2.9V 如果MCU VDD低于2.9V返回错误*/
	if (PWR->CSR & PWR_CSR_PVDO){
		Status = FLASH_ERROR_PROGRAM;
	}else{
		/* Write the variable virtual address and value in the EEPROM 在EEPROM中写入变量虚拟地址和值*/
		Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
		/* In case the EEPROM active page is full 万一EEPROM活动页面已满*/
		if (Status == PAGE_FULL){
			/* Perform Page transfer 执行页面转移*/
			Status = EE_PageTransfer(VirtAddress, Data);
		}
	}
	/* Return last operation status 返回上次操作状态*/
	return Status;
}

/**
 * @brief  Erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE
 * @param  None
 * @retval Status of the last operation (Flash write or erase) done during
 *         EEPROM formating
 */
static FLASH_Status EE_Format(void)
{
	FLASH_Status FlashStatus = FLASH_COMPLETE;
	/* Erase Page0 消除Page0*/
	FlashStatus = EE_EraseSectorIfNotEmpty(PAGE0_ID, VOLTAGE_RANGE);
	/* If erase operation was failed, a Flash error code is returned 如果擦除操作失败，则返回Flash错误码*/
	if (FlashStatus != FLASH_COMPLETE){
		return FlashStatus;
	}
	/* Set Page0 as valid page: Write VALID_PAGE at Page0 base address 将Page0设置为有效页面:在Page0基址写入VALID_PAGE*/
	FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
	/* If program operation was failed, a Flash error code is returned */
	if (FlashStatus != FLASH_COMPLETE){
		return FlashStatus;
	}
	/* Erase Page1 */
	FlashStatus = EE_EraseSectorIfNotEmpty(PAGE1_ID, VOLTAGE_RANGE);
	/* Return Page1 erase operation status */
	return FlashStatus;
}

/**
 * @brief  Find valid Page for write or read operation
 * @param  Operation: operation to achieve on the valid page.
 *   This parameter can be one of the following values:
 *     @arg READ_FROM_VALID_PAGE: read operation from valid page
 *     @arg WRITE_IN_VALID_PAGE: write operation from valid page
 * @retval Valid page number (PAGE or PAGE1) or NO_VALID_PAGE in case
 *   of no valid page was found
 */
static uint16_t EE_FindValidPage(uint8_t Operation)
{
	uint16_t PageStatus0 = 6, PageStatus1 = 6;
	/* Get Page0 actual status 获取Page0的实际状态*/
	PageStatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);
	/* Get Page1 actual status 获取Page1的实际状态*/
	PageStatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);
	/* Write or read operation 写或读操作*/
	switch (Operation){
		case WRITE_IN_VALID_PAGE:{   /* ---- Write operation 写操作---- */
			if (PageStatus1 == VALID_PAGE){			//0x0000
													/* Page0 receiving data */
				if (PageStatus0 == RECEIVE_DATA){
					return PAGE0;         			/* Page0 valid */
				}else{
					return PAGE1;         			/* Page1 valid */
				}
			}else if (PageStatus0 == VALID_PAGE){	//0x0000
				/* Page1 receiving data */
				if (PageStatus1 == RECEIVE_DATA){
					return PAGE1;         /* Page1 valid */
				}else{
					return PAGE0;         /* Page0 valid */
				}
			}else{
				return NO_VALID_PAGE;   /* No valid Page */
			}
		}
		case READ_FROM_VALID_PAGE:{  /* ---- Read operation ---- */
			if (PageStatus0 == VALID_PAGE){
				return PAGE0;           /* Page0 valid */
			}else if (PageStatus1 == VALID_PAGE){
				return PAGE1;           /* Page1 valid */
			}else{
				return NO_VALID_PAGE ;  /* No valid Page */
			}
		}
		default:
			return PAGE0;             /* Page0 valid */
	}
}

/**
 * @brief  Verify if active page is full and Writes variable in EEPROM.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data){
	FLASH_Status FlashStatus = FLASH_COMPLETE;
	uint16_t ValidPage = PAGE0;
	uint32_t Address = EEPROM_START_ADDRESS, PageEndAddress = EEPROM_START_ADDRESS+PAGE_SIZE;
	/* Get valid Page for write operation 获取写操作的有效页面*/
	ValidPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);
	/* Check if there is no valid page 检查是否没有有效的页面*/
	if (ValidPage == NO_VALID_PAGE){
		return  NO_VALID_PAGE;
	}
	/* Get the valid Page start Address */
	Address = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));
	/* Get the valid Page end Address */
	PageEndAddress = (uint32_t)((EEPROM_START_ADDRESS - 2) + (uint32_t)((1 + ValidPage) * PAGE_SIZE));
	/* Check each active page address starting from begining */
	while (Address < PageEndAddress){
		/* Verify if Address and Address+2 contents are 0xFFFFFFFF */
		if ((*(__IO uint32_t*)Address) == 0xFFFFFFFF){
			/* Set variable data */
			FlashStatus = FLASH_ProgramHalfWord(Address, Data);
			/* If program operation was failed, a Flash error code is returned */
			if (FlashStatus != FLASH_COMPLETE){
				return FlashStatus;
			}
			/* Set variable virtual address */
			FlashStatus = FLASH_ProgramHalfWord(Address + 2, VirtAddress);
			/* Return program operation status */
			return FlashStatus;
		}else{
			/* Next address location */
			Address = Address + 4;
		}
	}
	/* Return PAGE_FULL in case the valid page is full */
	return PAGE_FULL;
}

/**
 * @brief  Transfers last updated variables data from the full Page to
 *   an empty one.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data)
{
	FLASH_Status FlashStatus = FLASH_COMPLETE;
	uint32_t NewPageAddress = EEPROM_START_ADDRESS;
	uint16_t OldPageId=0;
	uint16_t ValidPage = PAGE0, VarIdx = 0;
	uint16_t EepromStatus = 0, ReadStatus = 0;

	/* Get active Page for read operation */
	ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

	if (ValidPage == PAGE1){       /* Page1 valid */
		/* New page address where variable will be moved to */
		NewPageAddress = PAGE0_BASE_ADDRESS;

		/* Old page ID where variable will be taken from */
		OldPageId = PAGE1_ID;
	}else if (ValidPage == PAGE0){  /* Page0 valid */
		/* New page address  where variable will be moved to */
		NewPageAddress = PAGE1_BASE_ADDRESS;

		/* Old page ID where variable will be taken from */
		OldPageId = PAGE0_ID;
	}else{
		return NO_VALID_PAGE;       /* No valid Page */
	}

	/* Set the new Page status to RECEIVE_DATA status */
	FlashStatus = FLASH_ProgramHalfWord(NewPageAddress, RECEIVE_DATA);
	/* If program operation was failed, a Flash error code is returned */
	if (FlashStatus != FLASH_COMPLETE){
		return FlashStatus;
	}

	/* Write the variable passed as parameter in the new active page */
	EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
	/* If program operation was failed, a Flash error code is returned */
	if (EepromStatus != FLASH_COMPLETE){
		return EepromStatus;
	}

	/* Transfer process: transfer variables from old to the new active page */
	for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++){
		if (VirtAddVarTab[VarIdx] != VirtAddress){  /* Check each variable except the one passed as parameter */
			/* Read the other last variable updates */
			ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
			/* In case variable corresponding to the virtual address was found */
			if (ReadStatus != 0x1){
				/* Transfer the variable to the new active page */
				EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
				/* If program operation was failed, a Flash error code is returned */
				if (EepromStatus != FLASH_COMPLETE){
					return EepromStatus;
				}
			}
		}
	}

	/* Erase the old Page: Set old Page status to ERASED status */
	FlashStatus = EE_EraseSectorIfNotEmpty(OldPageId, VOLTAGE_RANGE);
	/* If erase operation was failed, a Flash error code is returned */
	if (FlashStatus != FLASH_COMPLETE){
		return FlashStatus;
	}

	/* Set new Page status to VALID_PAGE status */
	FlashStatus = FLASH_ProgramHalfWord(NewPageAddress, VALID_PAGE);
	/* If program operation was failed, a Flash error code is returned */
	if (FlashStatus != FLASH_COMPLETE){
		return FlashStatus;
	}

	/* Return last operation flash status */
	return FlashStatus;
}

/*
 * Erase flash page if it is not already erased. This is to save write cycles and
 * prevent the memory from getting erased in case of unstable voltage at boot.
 */
static uint16_t EE_EraseSectorIfNotEmpty(uint32_t FLASH_Sector, uint8_t VoltageRange) {
	uint8_t *addr = flash_helper_get_sector_address(FLASH_Sector);
	for (unsigned int i = 0;i < PAGE_SIZE;i++) {
		if (addr[i] != 0xFF) {
			if(FLASH_Sector==PAGE0_ID || FLASH_Sector==PAGE1_ID){
				return FLASH_EraseSector(FLASH_Sector, VoltageRange);
			}
		}
	}
	return FLASH_COMPLETE;
}

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
