#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _MODEM_H_
	#define _MODEM_H_
	#include "hw_config.h"
	uint8_t replyProcessing(uint8_t *buff_p,uint8_t cmdType,uint16_t *gLength,uint8_t SioType_t);
	uint8_t sendProcessing(uint8_t *buff_p,uint8_t cmdType);
	float FloatMutualChar(uint8_t *u8Buf,float data,uint8_t Mode,uint8_t bBigEndian);
	uint8_t IntMutualChar(uint8_t *u8Buf,int data,uint8_t bBigEndian);
	void getcanConfigParam(canConfig *SrcDesStr);
	
	uint8_t InterfaceRelaid(const uint8_t *payload_t,uint8_t payload_len,uint32_t DevId_t,uint8_t relaidType);
	uint8_t registerComSendCallback(uint8_t(*comInterfa_CallBack_t)(void *payload),relaid_type_t value_t);
	void registerHwFlash_Write(uint8_t(*HwFlash_Write_t) (uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite));
	void registerHwFlash_Read(void(*HwFlash_Read_t) (uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite));
	void registerHwFlash_Erase(void(*HwFlash_Erase_t) (uint32_t WriteAddr));
	void registeMoveVoidFun(uint8_t funName,void(*MoveVoidFun_t)(void));
	void registeMoveUint8Fun(uint8_t funName,void(*MoveUint8Fun_t)(uint8_t status_t));
	void registeMoveCalTarget(uint8_t funName,uint8_t(*MoveCalFun_t) (float input,float rVel));
	void registeMoveServo( void(*MoveServoPerform_t)(uint16_t s1Data,uint16_t s2Data,uint16_t s3Data,uint16_t s4Data));
	void registeMoveLinkTwistSend(bool(*MoveLinkTwistSend_t)(const stcTwist *p));
	void registeMoveSetModeValue(uint8_t(*MoveSetModeValue_t)(stcIndepend src));
	void registeTerminalSend(void(*Send_t)(uint8_t *src_t,uint16_t srcLen));
	
	uint16_t upgraderProcessing(upGraderDate *wUpStruct,uint8_t *sioBuf_t,uint8_t type_t);
	uint16_t upgraderCrcProcessing(uint8_t *InputBuf,uint8_t type_t);
	uint8_t communicationSend_Struct(void *payload,uint8_t cmdTrpe);
	void bootUpErrorFirCopyApp(void);
	uint8_t bootFirCopyApp(void);
	void bootCheckUpgrader(void);
	void bootCheckApp(void);
	uint8_t appFirCopyApp(void);
	uint8_t appFirCopyBoot(void);
	void checkAppFirm(void);
	void checkUpgraderFirm(void);
	void appClearUpFalfg(void);
	void appSetUpFalfg(void);
	
	void Sys_printf(char* fmt,...); 
	void Terminal_printf(char* fmt,...);
	void Terminal_send(uint8_t *buf_t,uint16_t leng_t);
	void usbOS_printf(char* fmt,...);
	void u1OS_printf(char* fmt,...); 
	void u2OS_printf(char* fmt,...); 
	void u3OS_printf(char* fmt,...);
	void u4OS_printf(char* fmt,...);  
	void u5OS_printf(char* fmt,...); 
	void canOS_printf(char* fmt,...);
	//该注册函数开放给用户增加自己的指令功能,说明如下
	//src_t在处理时是串口接收过来的数据,用户可以直接使用
	//src_t在需要把数据传输回上位机时,可直接对src_t[srcLen+xx]进行幅值,例如传一个float数据则
	//src_t[srcLen+0] = float_b0 src_t[srcLen+1] = float_b1 src_t[srcLen+2] = float_b2 src_t[srcLen+3] = float_b3
	//srcLen在处理时是有效数据的起始位置即 PACK_DATA_INDEX
	//srcLen在处理完成后,如果需要把数据传回上位机,则应该是数据的有效长度,例如传一个float数据则srcLen = 4
	//返回值说明:1:保存参数,2:恢复默认参数,3:开启主动上报,4:禁用主动上报,5~10不可填写,9F,AF不可填写,
    //		     FE:无需回复数据给上位机,FF:下位机不支持该指令
	void registerCustomModenPrtFun(uint8_t(*CustomModenPrt_t)(uint8_t *src_t,uint16_t *srcLen));
	#endif 
#ifdef __cplusplus
}
#endif




