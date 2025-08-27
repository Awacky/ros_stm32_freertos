#ifdef __cplusplus
extern "C" {
#endif

#include "RB_C620.h"
#include "can.h"
void Set_Moto_Current_200(int16_t ESC_M1, int16_t ESC_M2, int16_t ESC_M3, int16_t ESC_M4)
{
	uint8_t Send_Data_Buff[8];
	Send_Data_Buff[0] = ESC_M1 >> 8;
	Send_Data_Buff[1] = ESC_M1;
	Send_Data_Buff[2] = ESC_M2 >> 8;
	Send_Data_Buff[3] = ESC_M2;
	Send_Data_Buff[4] = ESC_M3 >> 8;
	Send_Data_Buff[5] = ESC_M3;
	Send_Data_Buff[6] = ESC_M4 >> 8;
	Send_Data_Buff[7] = ESC_M4;	
//	CAN2_Send_Msg(0x200,0x200,Send_Data_Buff,8);
}	
void Set_Moto_Current_1FF(int16_t ESC_M5, int16_t ESC_M6, int16_t ESC_M7, int16_t ESC_M8)
{
	uint8_t Send_Data_Buff[8];
	Send_Data_Buff[0] = ESC_M5 >> 8;
	Send_Data_Buff[1] = ESC_M5;
	Send_Data_Buff[2] = ESC_M6 >> 8;
	Send_Data_Buff[3] = ESC_M6;
	Send_Data_Buff[4] = ESC_M7 >> 8;
	Send_Data_Buff[5] = ESC_M7;
	Send_Data_Buff[6] = ESC_M8 >> 8;
	Send_Data_Buff[7] = ESC_M8;	
//	CAN2_Send_Msg(0x1FF,0x1FF,Send_Data_Buff,8);
}	
#ifdef __cplusplus
}
#endif