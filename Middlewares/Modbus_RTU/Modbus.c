#ifdef __cplusplus
extern "C" {
#endif
	
#include "Modbus.h"
	
static uint8_t(*osSend_func)(void *getStcABuff) = 0;

void registeMBSend(uint8_t(*osSend_func_t)(void *getStcABuff)){
    osSend_func = osSend_func_t;
}	
static void RS485_Send_Data(unsigned char *send_buf,uint32_t length)
{	
	stcATBuff s_buff_t;
	s_buff_t.length_t = length;
	memcpy(s_buff_t.DataBuff,(char *)send_buf,length);
	if(osSend_func){
		osSend_func(&s_buff_t);
	}
}
/*************************
 * Name:    CRC-16/MODBUS x16+x15+x2+1
 * Poly:    0x8005
 * Init:    0xFFFF
 * Refin:   True
 * Refout:  True
 * Xorout:  0x0000
 * Note:
*************************/
static uint16_t crc16Modbus_table[256] = {
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};
/**
******************************************************************************
    ** \brief  modbus RTU CRC校验函数
    ** @param  buf 校验数据 ，len 数据长度
    ** \retval 无
    **
******************************************************************************/
uint16_t MB_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t tmp = 0;
    uint16_t crc = 0xFFFF;                // Initial value
    while(length--)
    {
        tmp =  (0x00FF & crc) ^ (*data++);    // crc ^= *data; data++;
        crc = (crc >> 8) ^ crc16Modbus_table[tmp&0xFF];
    }
    return crc; // crc
}

//unsigned int MB_CRC16(uint8_t *buf,  uint16_t len)
//{
//	unsigned int crc = 0xFFFF;
//	for (int pos = 0; pos < len; pos++){
//		crc ^= (unsigned int)buf[pos]; 
//		for (int i = 8; i != 0; i--){
//			if ((crc & 0x0001) != 0){
//				crc >>= 1; 
//				crc ^= 0xA001;
//			} else {
//				crc >>= 1;    
//			}
//		}
//	}
//	return crc;
//}
/** 
  * 函数功能: 读N个线圈状态(CoilStatue)
  * 输入参数: _addr:从站地址,_reg:寄存器地址,_num:待读取的数量
  * 返 回 值: 无
  * 说    明: 填充数据发送缓存区,然后发送
  */
void MB_ReadCoil_01H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	uint16_t TxCount = 0;
	uint16_t crc = 0;
	uint8_t Tx_Buf[8];
	Tx_Buf[TxCount++] = _addr;		    /* 从站地址 */
	Tx_Buf[TxCount++] = 0x01;		    /* 功能码 */	
	Tx_Buf[TxCount++] = _reg >> 8;	  	/* 寄存器地址 高字节 */
	Tx_Buf[TxCount++] = _reg;		    /* 寄存器地址 低字节 */
	Tx_Buf[TxCount++] = _num >> 8;	  	/* 线圈(bit)个数 高字节 */
	Tx_Buf[TxCount++] = _num;		    /* 线圈(bit)个数 低字节 */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
	Tx_Buf[TxCount++] = crc;	          /* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc 高字节 */
	RS485_Send_Data((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * 函数功能: 写单个线圈状态(CoilStatue)
  * 输入参数: _addr:从站地址,_reg:寄存器地址,_sta:待写入的线圈状态(0,1)
  * 返 回 值: 无
  * 说    明: 填充数据发送缓存区,然后发送
  */
void MB_WriteCoil_05H(uint8_t _addr, uint16_t _reg, uint16_t _sta)
{
	uint16_t TxCount = 0;
	uint16_t crc = 0;
	uint8_t Tx_Buf[8];
	Tx_Buf[TxCount++] = _addr;		    /* 从站地址 */
	Tx_Buf[TxCount++] = 0x05;		    /* 功能码 */	
	Tx_Buf[TxCount++] = _reg >> 8;	  	/* 寄存器地址 高字节 */
	Tx_Buf[TxCount++] = _reg;		    /* 寄存器地址 低字节 */
	Tx_Buf[TxCount++] = _sta >> 8;	  	/* 线圈(bit)个数 高字节 */
	Tx_Buf[TxCount++] = _sta;		    /* 线圈(bit)个数 低字节 */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
	Tx_Buf[TxCount++] = crc;	          	/* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		    /* crc 高字节 */
	RS485_Send_Data((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * 函数功能: 读输入状态状态(InputStatue)
  * 输入参数: _addr:从站地址,_reg:寄存器地址,_num:待读取的输入数量
  * 返 回 值: 无
  * 说    明: 填充数据发送缓存区,然后发送
  */
void MB_ReadInput_02H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	uint16_t TxCount = 0;
	uint16_t crc = 0;
	uint8_t Tx_Buf[8];
	Tx_Buf[TxCount++] = _addr;		    /* 从站地址 */
	Tx_Buf[TxCount++] = 0x02;		    /* 功能码 */	
	Tx_Buf[TxCount++] = _reg >> 8;	  	/* 寄存器地址 高字节 */
	Tx_Buf[TxCount++] = _reg;		    /* 寄存器地址 低字节 */
	Tx_Buf[TxCount++] = _num >> 8;	  	/* 开关(Input)个数 高字节 */
	Tx_Buf[TxCount++] = _num;		    /* 开关(Input)个数 低字节 */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
	Tx_Buf[TxCount++] = crc;	          	/* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		    /* crc 高字节 */
	RS485_Send_Data((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * 函数功能: 读保持寄存器(HoldingRegister)
  * 输入参数: _addr:从站地址,_reg:寄存器地址,_num:待读取的寄存器数量
  * 返 回 值: 无
  * 说    明: 填充数据发送缓存区,然后发送
  */
void MB_ReadHoldingReg_03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	uint16_t TxCount = 0;
	uint16_t crc = 0;
	uint8_t Tx_Buf[8];
	Tx_Buf[TxCount++] = _addr;		    /* 从站地址 */
	Tx_Buf[TxCount++] = 0x03;		    /* 功能码 */	
	Tx_Buf[TxCount++] = _reg >> 8;	  	/* 寄存器地址 高字节 */
	Tx_Buf[TxCount++] = _reg;		    /* 寄存器地址 低字节 */
	Tx_Buf[TxCount++] = _num >> 8;	  	/* 寄存器(16bits)个数 高字节 */
	Tx_Buf[TxCount++] = _num;		    /* 低字节 */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
	Tx_Buf[TxCount++] = crc;	        /* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		    /* crc 高字节 */
	RS485_Send_Data((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * 函数功能: 读N个输入寄存器(InputRegister)
  * 输入参数: _addr:从站地址,_reg:寄存器地址,_num:待读取的寄存器数量
  * 返 回 值: 无
  * 说    明: 填充数据发送缓存区,然后发送.
  */
void MB_ReadInputReg_04H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	uint16_t TxCount = 0;
	uint16_t crc = 0;
	uint8_t Tx_Buf[8];
	Tx_Buf[TxCount++] = _addr;		    /* 从站地址 */
	Tx_Buf[TxCount++] = 0x04;		    /* 功能码 */	
	Tx_Buf[TxCount++] = _reg >> 8;	  	/* 寄存器地址 高字节 */
	Tx_Buf[TxCount++] = _reg;		    /* 寄存器地址 低字节 */
	Tx_Buf[TxCount++] = _num >> 8;	  	/* 寄存器(16bits)个数 高字节 */
	Tx_Buf[TxCount++] = _num;		    /*  低字节 */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
	Tx_Buf[TxCount++] = crc;	          /* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc 高字节 */
	RS485_Send_Data((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * 函数功能: 写单个保持寄存器(HoldingRegister)
  * 输入参数: _addr:从站地址,_reg:寄存器地址,_data:待写入的寄存器数据
  * 返 回 值: 无
  * 说    明: 填充数据发送缓存区,然后发送
  */
void MB_WriteHoldingReg_06H(uint8_t _addr, uint16_t _reg, uint16_t _data)
{
	uint16_t TxCount = 0;
	uint16_t crc = 0;
	uint8_t Tx_Buf[8];
	Tx_Buf[TxCount++] = _addr;		    /* 从站地址 */
	Tx_Buf[TxCount++] = 0x06;		    /* 功能码 */	
	Tx_Buf[TxCount++] = _reg >> 8;	  	/* 寄存器地址 高字节 */
	Tx_Buf[TxCount++] = _reg;		    /* 寄存器地址 低字节 */
	Tx_Buf[TxCount++] = _data >> 8;	  	/* 寄存器(16bits)个数 高字节 */
	Tx_Buf[TxCount++] = _data;		    /*  低字节 */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
	Tx_Buf[TxCount++] = crc;	          /* crc 低字节 */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc 高字节 */
	RS485_Send_Data((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * 函数功能: 写N个保持寄存器(HoldingRegister)
  * 输入参数: _addr:从站地址,_reg:寄存器地址,_num:待写入的寄存器数量,_databuf:待写入的寄存器数据
  * 返 回 值: 无
  * 说    明: 填充数据发送缓存区,然后发送._databuf的长度需 >= _num*2
  */
void MB_WriteNumHoldingReg_10H(uint8_t _addr, uint16_t _reg, uint16_t _num,uint8_t *_databuf)
{
	uint16_t i;
	uint16_t TxCount = 0;
	uint16_t crc = 0;
	uint8_t *Tx_BufNew = (uint8_t *)malloc(10+2*_num);
	Tx_BufNew[TxCount++] = _addr;		    /* 从站地址 */
	Tx_BufNew[TxCount++] = 0x10;		    /* 功能码 */	
	Tx_BufNew[TxCount++] = _reg >> 8;	  	/* 寄存器地址 高字节 */
	Tx_BufNew[TxCount++] = _reg;		    /* 寄存器地址 低字节 */
	Tx_BufNew[TxCount++] = _num >> 8;	  	/* 寄存器(16bits)个数 高字节 */
	Tx_BufNew[TxCount++] = _num;		    /* 低字节 */
	Tx_BufNew[TxCount++] = _num<<1;		  	/* 数据个数 */
	for (i = 0; i < 2 * _num; i++) {
		Tx_BufNew[TxCount++]  = _databuf[i];	/* 后面的数据长度 */
	}
	crc = MB_CRC16((uint8_t*)&Tx_BufNew,TxCount);
	Tx_BufNew[TxCount++] = crc;	          		/* crc 低字节 */
	Tx_BufNew[TxCount++] = crc>>8;		      	/* crc 高字节 */
	RS485_Send_Data((uint8_t *)&Tx_BufNew,TxCount);
	free(Tx_BufNew);
}
#ifdef __cplusplus
}
#endif
