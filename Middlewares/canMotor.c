#ifdef __cplusplus
extern "C" {
#endif
#include "canMotor.h"
static _Moto_Str *canDev_Rpm = NULL;
static void(*canDelay_func)(uint16_t daley_t) = NULL;
static bool canMotorInitStatus = false;
void sdoWriteMotorSpin(vMotorStr *Src_t,uint8_t EnOsSend_t){
	canStrMsg src_prt;
	memset(&src_prt,0,sizeof(canStrMsg));
	int16_t sValue = Src_t->dutyVal*Src_t->mDir;
	src_prt.Data[0] = 0x23;
	src_prt.Data[1] = 0x00;
	src_prt.Data[2] = 0x20;
	src_prt.Data[3] = (Src_t->mName + 1);
	src_prt.Data[4] = sValue;
	src_prt.Data[5] = sValue>>8;
	src_prt.Data[6] = 0;
	src_prt.Data[7] = 0;
	src_prt.DLC = 8;
	src_prt.StdId = (0x600|Src_t->mId);	
	if(EnOsSend_t){
		#ifndef Custom
		communicationSend_Struct(&src_prt,CAN_Enabled);
		#else
		
		#endif
	} else {
		#ifndef Custom
		communicationSend_Struct(&src_prt,RELAID_RESERVE1);
		#else
		
		#endif
	}
}
void sdoReadCanMotorRpm(vMotorStr *Src_t,uint8_t EnOsSend_t){
	canStrMsg src_prt;
	memset(&src_prt,0,sizeof(canStrMsg));
	int16_t sValue = Src_t->dutyVal*Src_t->mDir;
	src_prt.Data[0] = 0x40;
	src_prt.Data[1] = 0x0A;
	src_prt.Data[2] = 0x21;
	src_prt.Data[3] = (Src_t->mName + 1);
	src_prt.DLC = 8;
	src_prt.StdId = (0x600|Src_t->mId);
	if(EnOsSend_t){
		#ifndef Custom
		communicationSend_Struct(&src_prt,CAN_Enabled);
		#else
		
		#endif
	} else {
		#ifndef Custom
		communicationSend_Struct(&src_prt,RELAID_RESERVE1);
		#else
		
		#endif
	}
}
void sdoReadCanMotorEncoder(vMotorStr *Src_t,uint8_t EnOsSend_t){
	canStrMsg src_prt;
	memset(&src_prt,0,sizeof(canStrMsg));
	int16_t sValue = Src_t->dutyVal*Src_t->mDir;
	src_prt.Data[0] = 0x40;
	src_prt.Data[1] = 0x04;
	src_prt.Data[2] = 0x21;
	src_prt.Data[3] = (Src_t->mName + 1);
	src_prt.DLC = 8;
	src_prt.StdId = (0x600|Src_t->mId);
	if(EnOsSend_t){
		#ifndef Custom
		communicationSend_Struct(&src_prt,CAN_Enabled);
		#else
		
		#endif
	} else {
		#ifndef Custom
		communicationSend_Struct(&src_prt,RELAID_RESERVE1);
		#else
		
		#endif
	}
}
void sdoReadCanMotorBat(vMotorStr *Src_t,uint8_t EnOsSend_t){
	canStrMsg src_prt;
	memset(&src_prt,0,sizeof(canStrMsg));
	int16_t sValue = Src_t->dutyVal*Src_t->mDir;
	src_prt.Data[0] = 0x40;
	src_prt.Data[1] = 0x0D;
	src_prt.Data[2] = 0x21;
	src_prt.Data[3] = (Src_t->mName + 1);
	src_prt.DLC = 8;
	src_prt.StdId = (0x600|Src_t->mId);
	if(EnOsSend_t){
		#ifndef Custom
		communicationSend_Struct(&src_prt,CAN_Enabled);
		#else
		
		#endif
	} else {
		#ifndef Custom
		communicationSend_Struct(&src_prt,RELAID_RESERVE1);
		#else
		
		#endif
	}
}
void setAllModeStop_Cmd(vMotorStr *Src_t){
	canStrMsg src_prt;
	memset(&src_prt,0,sizeof(canStrMsg));
	src_prt.Data[0] = 0x2F;
	src_prt.Data[1] = 0x0E;
	src_prt.Data[2] = 0x20;
	src_prt.Data[3] = 0;
	src_prt.DLC = 4;
	src_prt.StdId = (0x600|Src_t->mId);
	#ifndef Custom
	communicationSend_Struct(&src_prt,RELAID_RESERVE1);
	#else
		
	#endif
}
void canMotorFeedbackProcessing(const canStrMsg *src){
	if(!canMotorInitStatus)return;
	if(0x181 == src->StdId){		//���ö�ȡRPM
		if(canDev_Rpm!=NULL){
			static uint16_t m16Data = 0;
			m16Data = src->Data[0];
			m16Data |= src->Data[1]<<8;
			canDev_Rpm->Current_Rpm1 = m16Data;
			m16Data = src->Data[2];
			m16Data |= src->Data[3]<<8;
			canDev_Rpm->Current_Rpm2 = m16Data;
		}
	}
	if(0x281 == src->StdId ){		//���ö�ȡEncoder
		if(canDev_Rpm!=NULL){
			static uint32_t m32Data = 0;
			m32Data  = src->Data[0];
			m32Data |= src->Data[1]<<8;
			m32Data |= src->Data[2]<<16;
			m32Data |= src->Data[3]<<24;
			canDev_Rpm->MotorEncoder1 = m32Data;
			m32Data  = src->Data[4];
			m32Data |= src->Data[5]<<8;
			m32Data |= src->Data[6]<<16;
			m32Data |= src->Data[7]<<24;
			canDev_Rpm->MotorEncoder2 = m32Data;
		}
	}
	if(0x581 == src->StdId){
		if(0x0A == src->Data[1] && 0x21 == src->Data[2] && 0x01 == src->Data[3]){
			static int16_t m16Data = 0;
			m16Data = src->Data[4];
			m16Data |= src->Data[5]<<8;
			canDev_Rpm->Current_Rpm1 = m16Data;
		}
		if(0x0A == src->Data[1] && 0x21 == src->Data[2] && 0x02 == src->Data[3]){
			static int16_t m16Data = 0;
			m16Data = src->Data[4];
			m16Data |= src->Data[5]<<8;
			canDev_Rpm->Current_Rpm2 = m16Data;
		}
		if(0x04 == src->Data[1] && 0x21 == src->Data[2] && 0x01 == src->Data[3]){
			static int32_t m32Data = 0;
			m32Data  = src->Data[4];
			m32Data |= src->Data[5]<<8;
			m32Data |= src->Data[6]<<16;
			m32Data |= src->Data[7]<<24;
			canDev_Rpm->MotorEncoder1 = m32Data;
		}
		if(0x04 == src->Data[1] && 0x21 == src->Data[2] && 0x02 == src->Data[3]){
			static int32_t m32Data = 0;
			m32Data  = src->Data[4];
			m32Data |= src->Data[5]<<8;
			m32Data |= src->Data[6]<<16;
			m32Data |= src->Data[7]<<24;
			canDev_Rpm->MotorEncoder2 = m32Data;
		}
	}
}
void canMotor_ConfigTPDO(void){
	canStrMsg src_prt;
	memset(&src_prt,0,sizeof(canStrMsg));
	uint8_t initMotor_Frame[28][8] = {    
	//дָ��(u8):0x23->u32,0x27->u24,0x2B->u16,0x2F->u8 Ӧ��:0x60->�ɹ�,0x80->ʧ�� 
	//TPDO ͨ�Ų�������
	//дָ��(u8),ӳ������(u16),������(u8),���ݴ�С(u8),������(u8),ӳ�䵽����(u16)
	//������(u8):01h COB-ID/PDO��ַ(U32),02h ��������(U8),03h ������ֹԼ��ʱ��(U16),05h �¼���ʱ������ʱ��(U16),06h syncͬ����ʼֵ(U8)
	//02h ��������(U8):1~240Ϊͬ������(���ٸ�SYNC),254��ʱ����,255�¼�����
	//TPDO ӳ���������
	//дָ��(u8),ӳ������(u16),������(u8),���ݴ�С(u8),������(u8),ӳ�䵽����(u16)
	//������(u8):	00h ʹ��/ʧ��(u8:0ʧ��/!0ʹ��),������Ŀ����
	//���ݴ�С(u8):0x20->u32,0x10->u16
		{0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},//TPDO1 ʧ�� 181
		{0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x0A, 0x21},//TPDO1 INDEX 2001h����Ϊ�ϴ�RPM M1(0x210A_01)
		{0x23, 0x00, 0x1A, 0x02, 0x10, 0x00, 0x0A, 0x21},//TPDO1 INDEX 2003h����Ϊ�ϴ�RPM M2(0x210A_02)
		{0x23, 0x00, 0x1A, 0x03, 0x00, 0x00, 0x00, 0x00},//TPDO2 INDEX 2005h����Ϊ00
		{0x2F, 0x00, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00},//TPDO1 ʹ�� 2��
		{0x2F, 0x00, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00},//TPDO1 �������� 1��SYNC
		{0x2F, 0x00, 0x18, 0x03, 0x02, 0x00, 0x00, 0x00},//TPDO1 ������ֹԼ��ʱ��2ms
		{0x2F, 0x00, 0x18, 0x05, 0x02, 0x00, 0x00, 0x00},//TPDO1 �¼���ʱ������ʱ��2ms

		{0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},//TPDO2 ʧ�� 281
		{0x23, 0x01, 0x1A, 0x01, 0x20, 0x00, 0x08, 0x21},//TPDO2 INDEX 2001h����Ϊ�ϴ��������������ֵM1(0x2108_01)
		{0x23, 0x01, 0x1A, 0x02, 0x20, 0x01, 0x08, 0x21},//TPDO2 INDEX 2003h����Ϊ�ϴ��������������ֵM2(0x2108_02)
		{0x2F, 0x01, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00},//TPDO2 ʹ��  2��
		{0x2F, 0x01, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00},//TPDO2 �������� 1��SYNC
		{0x2F, 0x01, 0x18, 0x03, 0x02, 0x00, 0x00, 0x00},//TPDO2 ������ֹԼ��ʱ��2ms
		{0x2F, 0x01, 0x18, 0x05, 0x02, 0x00, 0x00, 0x00},//TPDO2 �¼���ʱ������ʱ��2ms

		{0x2F, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},//TPDO1 ʧ�� 381
		{0x23, 0x02, 0x1A, 0x01, 0x10, 0x00, 0x0D, 0x21},//TPDO1 INDEX 2001h����Ϊ�ϴ�RPM M1(0x210D_01)
		{0x23, 0x02, 0x1A, 0x02, 0x10, 0x00, 0x0D, 0x21},//TPDO1 INDEX 2003h����Ϊ�ϴ�RPM M2(0x210D_02)
		{0x23, 0x02, 0x1A, 0x03, 0x00, 0x00, 0x00, 0x00},//TPDO2 INDEX 2005h����Ϊ00
		{0x2F, 0x02, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00},//TPDO1 ʹ�� 2��
		{0x2F, 0x02, 0x18, 0x02, 0x01, 0x00, 0x00, 0x00},//TPDO1 �������� 1��SYNC
		{0x2F, 0x02, 0x18, 0x03, 0xC8, 0x00, 0x00, 0x00},//TPDO1 ������ֹԼ��ʱ��200ms
		{0x2F, 0x02, 0x18, 0x05, 0xC8, 0x00, 0x00, 0x00},//TPDO1 �¼���ʱ������ʱ��200ms

		{0x2F, 0x00, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},//RPDO1 ��������Ϊ�첽����
		{0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},//RPDO1 ʧ�� 201
		{0x23, 0x00, 0x16, 0x01, 0x10, 0x01, 0x00, 0x20},//RPDO1 ����ΪM1�ٶ�(S16)
		{0x23, 0x00, 0x16, 0x02, 0x10, 0x02, 0x00, 0x20},//RPDO1 ����ΪM2�ٶ�(S16)
		{0x2F, 0x00, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},//RPDO1 ʹ��  2��
	};
	src_prt.ExtId = 0;
	src_prt.StdId = 0x601;
	src_prt.DLC = 8;
	for(uint8_t k=0;k<28;k++){
		memcpy(src_prt.Data,initMotor_Frame[k],8);
		if(canDelay_func!=NULL){
			canDelay_func(5);
		}
		#ifndef Custom
		communicationSend_Struct(&src_prt,CAN_Enabled);
		#endif
	}
}
void canMotor_OpenPDO(const uint8_t id) {//ʹ��PDO
	canStrMsg src_prt;
	memset(&src_prt,0,sizeof(canStrMsg));
	src_prt.Data[0] = 0x01;
	src_prt.Data[1] = id;
	src_prt.ExtId = 0;
	src_prt.StdId = 0;
	src_prt.DLC = 2;
	#ifndef Custom
	communicationSend_Struct(&src_prt,CAN_Enabled);
	#else
		
	#endif
}
void canMotorInit(vMotorStr *Src_t){
	canMotor_OpenPDO(1);
	if(canDelay_func!=NULL){
		canDelay_func(5);
	}
	canMotor_ConfigTPDO();
	canMotorInitStatus = true;
}
void canMotorSpin(vMotorStr * M1Value,vMotorStr * M2Value){
	if(!canMotorInitStatus)return;
	int16_t sValue = 0;
	canStrMsg src_prt;
	memset(&src_prt,0,sizeof(canStrMsg));
	sValue = M1Value->dutyVal * M1Value->mDir;
	src_prt.Data[0] = sValue;
	src_prt.Data[1] = sValue>>8;
	sValue = M2Value->dutyVal * M2Value->mDir;
	src_prt.Data[2] = sValue;
	src_prt.Data[3] = sValue>>8;
	src_prt.Data[4] = 0;
	src_prt.Data[5] = 0;
	src_prt.Data[6] = 0;
	src_prt.Data[7] = 0;
	src_prt.DLC = 8;
	src_prt.StdId = 0x201;
	#ifndef Custom
	communicationSend_Struct(&src_prt,RELAID_RESERVE1);
	#else
		
	#endif
}
void registeCanDelayFun(void(*canDelay_t)(uint16_t daley_t)){
	canDelay_func = canDelay_t;
}
void registeCanMotorValueFun(_Moto_Str *canDevRpm_t){
	canDev_Rpm = canDevRpm_t;
}
//void get

#ifdef __cplusplus
}
#endif








