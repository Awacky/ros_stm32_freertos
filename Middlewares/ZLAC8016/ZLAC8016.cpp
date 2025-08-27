#ifdef __cplusplus

#include "ZLAC8016.h"
#include "RS485.h"
ZLAC8016::ZLAC8016()
{

}
void ZLAC8016::UpdateParam(u8 Dev_Id)
{
	this->DevId = Dev_Id;
}	
void ZLAC8016::DeviceModeSet(void)
{
	//01 06 20 32 00 03 63 C4
	MB_WriteHoldingReg_06H(DevId,0x2032,0x03);
}
void ZLAC8016::DeviceSpeedTimeSet(void)
{
	MB_WriteHoldingReg_06H(DevId,0x2037,0x01F4);	//����ʱ��500ms
//	delay_xms(1);
	MB_WriteHoldingReg_06H(DevId,0x2037,0x01F4);	//����ʱ��500ms
}
void ZLAC8016::DeviceEnabled(void)
{
	MB_WriteHoldingReg_06H(DevId,0x2031,0x08); 		//���ʹ��
}
void ZLAC8016::DeviceSpeedSet(int MotorRpm)			//�ٶ�����RPM
{
	MB_WriteHoldingReg_06H(DevId,0x203A,MotorRpm);	//
}
void ZLAC8016::DeviceDisability(void)				//���ʧ��
{
	MB_WriteHoldingReg_06H(DevId,0x2031,0x07);		//
}
void ZLAC8016::DeviceReadInfo(void)					//��ȡ��������
{
	MB_ReadHoldingReg_03H(DevId,0x2026,0x0A);
}

#endif


