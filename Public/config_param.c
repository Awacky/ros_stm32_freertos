#include "config_param.h"
#include "device_storage.h"
#include "eeprom.h"
#include "osQueueSemap.h"
#include "ErrorManage_Task.h"

// EEPROM settings
#define EEPROM_BASE_UP_PARAM	2000
#define EEPROM_BASE_APP_PARAM	1000

uint16_t VirtAddVarTab[NB_OF_VAR];
static uint32_t gLength = 0;
Param_Struct configParam;
static SemaphoreHandle_t  Wx_Semaphore = NULL;
static uint8_t configParamCksum(Param_Struct* data,uint32_t length);
static void configParamTask(void* param);
static void getDefaultParam(Param_Struct *Defconf);
static Param_Struct ParamDefault=
{
	.RobotType     = ROBOT_D4,
	.MotorType     = M_BTN797X,
	.IMUType       = I_MPU6050_N,	//
	.ROS_SERIALx   = SERIAL1,		//Serial_TypeDef
	.ROSType	   = 0,				//0:ROS1 1:ROS2
	.MotorDebug    = 0,
	.ControlMode   = 0,
	.InfoEnabled   = 0,
	.WS2812Enabled = 0,
	.perEnabled ={
		.servo1 = 0,
		.servo2 = 0,
		.servo3 = 0,
		.servo4 = 0,
		.rgbled = 0,
		.lowBat = 0,
	},
	.PubVel_Hz     = 20,
	.PubImu_Hz     = 40,
	.PubBat_Hz     = 5,
	.PubSonar_Hz   = 15,
	.TurnInitAngle = 0,
	.MaxSteerAngle = 60,
	.EscMedianValue= 1500,
	.Max_RPM       = 360,
	.Max_PWM       = 200,
	.Pwm_psc       = 84,
	.Pwm_arr	   = 200,
	.WheelDiameter = 0.095,
	.LRInterval    = 0.25,
	.FBInterval    = 0.1,
	.CountsPer	   = 1560.0,
	.Max_Speed     = 1.0,
	.RobotRadius   = 0.2,
	.p_M1.K_p	   = 0.5,
	.p_M1.K_i	   = 0.3,
	.p_M1.K_d      = 0.1,
	.p_M2.K_p	   = 0.5,
	.p_M2.K_i	   = 0.3,
	.p_M2.K_d      = 0.1,
	.p_M3.K_p	   = 0.5,
	.p_M3.K_i	   = 0.3,
	.p_M3.K_d      = 0.1,
	.p_M4.K_p	   = 0.5,
	.p_M4.K_i	   = 0.3,
	.p_M4.K_d      = 0.1,
	.M1 =
	{
		.Motor		= 1,
		.Encoder 	= 1,
		.Id			= 1,
	},
	.M2 =
	{
		.Motor 		= 1,
		.Encoder 	= 1,
		.Id			= 1,
	},
	.M3 =
	{
		.Motor 		= 1,
		.Encoder 	= 1,
		.Id			= 1,
	},
	.M4 =
	{
		.Motor 		= 1,
		.Encoder 	= 1,
		.Id			= 1,
	},
	.uart1Cfg=
	{
		.Port_BaudRate = 115200,
		.Port_WordLength = 0,	//8位数据位
		.Port_StopBits=10,		//1位停止位
		.Port_Parity=0,     	//无校验位
		.Port_Mode=0,       	//收发模式	
		.Port_HFC=0,			//无流控
	},
	.uart2Cfg=
	{
		.Port_BaudRate = 9600,
		.Port_WordLength = 0,	//8位数据位
		.Port_StopBits=10,		//1位停止位
		.Port_Parity=0,     	//无校验位
		.Port_Mode=0,       	//收发模式	
		.Port_HFC=0,			//无流控
	},
	.uart3Cfg=
	{
		.Port_BaudRate = 115200,
		.Port_WordLength = 0,	//8位数据位
		.Port_StopBits=10,		//1位停止位
		.Port_Parity=0,     	//无校验位
		.Port_Mode=0,       	//收发模式	
		.Port_HFC=0,			//无流控
	},
	.uart4Cfg=
	{
		.Port_BaudRate = 100000,
		.Port_WordLength = 0,	//8位数据位
		.Port_StopBits=20,		//2位停止位
		.Port_Parity=1,     	//无校验位
		.Port_Mode=0,       	//收发模式	
		.Port_HFC=0,			//无流控
	},
	.uart5Cfg=
	{
		.Port_BaudRate = 115200,
		.Port_WordLength = 0,	//8位数据位
		.Port_StopBits=10,		//1位停止位
		.Port_Parity=0,     	//无校验位
		.Port_Mode=0,       	//收发模式	
		.Port_HFC=0,			//无流控
	},
	.canSetParam=
	{
		.BRP_t=0,
		.RSAW_t=0,
		.BTS1_t=0,
		.BTS2_t=0,
		.BaudRate=0,
		.APB_Clk=0,
	},
	.sonarCfg=
	{
		.En1 = 0,							//1 脉冲方式 2 Modbus
		.En2 = 0,							//1 脉冲方式 2 Modbus
		.En3 = 0,							//1 脉冲方式 2 Modbus
		.En4 = 0,							//1 脉冲方式 2 Modbus
		.DeceDist1 = 400,
		.DeceDist2 = 400,
		.DeceDist3 = 400,
		.DeceDist4 = 400,
		.EstopDist1= 200,
		.EstopDist2= 200,
		.EstopDist3= 200,
		.EstopDist4= 200,
	},
	.wifiNum = 0xFF,
	.IsUsbLink = 0,
	.IsRosNodePub = 0,
	.cksum = 0xFF,
};
static uint8_t configParamCksum(Param_Struct* data,uint32_t length){
	int i;
	uint8_t cksum=0;	
	uint8_t* c = (uint8_t*)data;
	for (i=0; i<length; i++){
		cksum += *(c++);
		if(c==(&data->cksum)){
			return cksum;
		}
	}
	cksum-=data->cksum;
	return cksum;
}
void readParamCfg(Param_Struct *conf) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;
	for (unsigned int i = 0;i < (sizeof(Param_Struct) / 2);i++) {
		if (EE_ReadVariable(EEPROM_BASE_APP_PARAM + i, &var) == 0) {
			conf_addr[2 * i] = (var >> 8) & 0xFF;
			conf_addr[2 * i + 1] = var & 0xFF;
		} else {
			is_ok = false;
			break;
		}
	}
	if (!is_ok) {
		getDefaultParam(conf);
	}
}
bool writeParamCfg(const Param_Struct *conf) {
	bool is_ok = false;
	uint16_t Status = 0;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;
	FLASH_Interface_UnlockClearFlag();
	for (unsigned int i = 0;i < (sizeof(Param_Struct) / 2);i++) {
		var = (conf_addr[2 * i] << 8) & 0xFF00;
		var |= conf_addr[2 * i + 1] & 0xFF;
		Status = EE_WriteVariable(EEPROM_BASE_APP_PARAM + i, var);
//		if (EE_WriteVariable(EEPROM_BASE_APP_PARAM + i, var) != FLASH_COMPLETE) {
		if (Status != FLASH_COMPLETE) {
			is_ok = true;
			break;
		}
	}
	FLASH_Interface_Lock();
	return is_ok;
}
static void configParamTask(void* param){	//参数配置任务
	uint8_t gCksum = 0;
	while(1) {	
		if(Wx_Semaphore!=NULL){
			xSemaphoreTake(Wx_Semaphore, portMAX_DELAY);
			configParam.cksum = 0;
//			gLength = PackNum(sizeof(Param_Struct),4);
			gCksum = configParamCksum(&configParam,sizeof(Param_Struct));
			if(configParam.cksum != gCksum){
				configParam.cksum = gCksum;	/*数据校验*/				
//				gCksum = STMFLASH_Write(APP_PARAM_ADDR,(uint32_t *)&configParam, gLength);	/*写入stm32 flash*/
				gCksum = writeParamCfg(&configParam);
				if(gCksum){
					setError_Fun(SAVE_PARAM_Er);
				}else{
					STARBOT_BEEP_On();
					vTaskDelay(150);
					STARBOT_BEEP_Off();
				}
			}		
		}else{
			vTaskDelay(1000);
		}				
	}	
}
void configParamInit(void)	/*参数配置初始化*/
{	
	uint8_t gCksum = 0;
	#ifndef Custom
	registerHwFlash_Write(STMFLASH_Write);	//用于升级和加密写入使用
	registerHwFlash_Read(STMFLASH_Read);	//用于升级读取使用
	registerHwFlash_Erase(STMFLASH_Erase);	//用于升级读取使用
	#endif
	memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));
	memset(&configParam,0,sizeof(Param_Struct));
	int ind = 0;
//	for (unsigned int i = 0;i < (sizeof(upGraderFlash) / 2);i++) {
//		VirtAddVarTab[ind++] = EEPROM_BASE_UP_PARAM + i;
//	}
	for (unsigned int i = 0;i < (sizeof(Param_Struct) / 2);i++) {
		VirtAddVarTab[ind++] = EEPROM_BASE_APP_PARAM + i;
	}
	FLASH_Interface_UnlockClearFlag();
	ind = EE_Init();
	FLASH_Interface_Lock();
	gLength = sizeof(Param_Struct);
//	gLength = PackNum(sizeof(Param_Struct),4);
//	STMFLASH_Read(APP_PARAM_ADDR, (uint32_t *)&configParam,gLength);
	readParamCfg(&configParam);									//APP程序不是从0x8008000以后开始则该函数需要注释，或者自己实现读取函数																						
	gCksum = configParamCksum(&configParam,sizeof(Param_Struct));
	if(configParam.FirstWrite != 0x1F || (configParam.FirstWrite == 0x1F && gCksum!=configParam.cksum)){ //首次写入参数或者参数校验失败	
		memcpy((uint8_t *)&configParam, (uint8_t *)&ParamDefault,sizeof(Param_Struct));
		configParam.FirstWrite = 0x1F;
		configParam.cksum = configParamCksum(&configParam,sizeof(Param_Struct));	
		ind = writeParamCfg(&configParam);						//APP程序不是从0x8008000以后开始则该函数需要注释，或者自己实现写入函数
//		ind = STMFLASH_Write(APP_PARAM_ADDR,(uint32_t *)&configParam,gLength);
		if(ind){
			setError_Fun(SAVE_PARAM_Er);
		}else{
			STARBOT_BEEP_On();
			delay_ms(150);
			STARBOT_BEEP_Off();
		}
	}	
	Wx_Semaphore = xSemaphoreCreateBinary();
	xTaskCreate(configParamTask, "CONFIG_TASK", 128, NULL,configParam_Pri, NULL);
}
void restoreDefaultParam(void){
	memcpy((uint8_t *)&configParam, (uint8_t *)&ParamDefault, sizeof(Param_Struct));
}
static void getDefaultParam(Param_Struct *Defconf){
	memcpy((uint8_t*)Defconf, (uint8_t *)&ParamDefault, sizeof(Param_Struct));
}
void configParamGiveSemaphore(void){	
	if(Wx_Semaphore!=NULL){
		xSemaphoreGive(Wx_Semaphore);
	}
		
}
