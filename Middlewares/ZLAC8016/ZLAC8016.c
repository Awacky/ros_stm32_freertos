#ifdef __cplusplus
extern "C" {
#endif

#include "ZLAC8016.h"
#include "RS485.h"
void DeviceModeSet(u8 DevId)
{
	//01 06 20 32 00 03 63 C4
	MB_WriteHoldingReg_06H(DevId,0x2032,0x03);
}
void DeviceSpeedTimeSet(u8 DevId)
{
	MB_WriteHoldingReg_06H(DevId,0x2037,0x01F4);//����ʱ��500ms
	delay_xms(1);
	MB_WriteHoldingReg_06H(DevId,0x2037,0x01F4);//����ʱ��500ms
}
void DeviceEnabled(u8 DevId)
{
	MB_WriteHoldingReg_06H(DevId,0x2031,0x08); //���ʹ��
}
void DeviceSpeedSet(u8 DevId,int MotorRpm)
{
	MB_WriteHoldingReg_06H(DevId,0x203A,MotorRpm);//����ʱ��500ms
}
void DeviceDisability(u8 DevId)
{
	MB_WriteHoldingReg_06H(DevId,0x2031,0x07);//����ʱ��500ms
}
void DeviceReadInfo(u8 DevId,)
{
	MB_ReadHoldingReg_03H(DevId,0x2026,0x08);
}
#ifdef __cplusplus
}
#endif


