#ifdef __cplusplus
extern "C" {
#endif
#ifndef __PSTWO_H
#define __PSTWO_H

	#include "hw_config.h"

	//These are our button constants
	#define PSB_SELECT      1
	#define PSB_L3          2
	#define PSB_R3          3
	#define PSB_START       4
	#define PSB_PAD_UP      5
	#define PSB_PAD_RIGHT   6
	#define PSB_PAD_DOWN    7
	#define PSB_PAD_LEFT    8
	#define PSB_L2          9
	#define PSB_R2          10
	#define PSB_L1          11
	#define PSB_R1          12
	#define PSB_GREEN       13
	#define PSB_RED         14
	#define PSB_BLUE        15
	#define PSB_PINK        16

	#define PSB_TRIANGLE    13
	#define PSB_CIRCLE      14
	#define PSB_CROSS       15
	#define PSB_SQUARE      16

	//#define WHAMMY_BAR		8

	//These are stick values
	#define PSS_RX 5                //右摇杆X轴数据
	#define PSS_RY 6
	#define PSS_LX 7
	#define PSS_LY 8


	typedef struct __PSTwo_Data_
	{
		int PS2_LX;
		int PS2_LY;
		int PS2_RX;
		int PS2_RY;
		int PS2_KEY;
	}PSTwo_Data;
	uint8_t PS2_RedLight(void);                   		//判断是否为红灯模式
	void PS2_ReadData(void);                 			//读手柄数据
	void PS2_Cmd(uint8_t CMD);		             		//向手柄发送命令
	uint8_t PS2_DataKey(void);		             		//按键值读取
	uint8_t PS2_AnologData(uint8_t button);            	//得到一个摇杆的模拟量
	void PS2_ClearData(void);	             			//清除数据缓冲区
	void PS2_Vibration(uint8_t motor1, uint8_t motor2);	//振动设置motor1  0xFF开，其他关，motor2  0x40~0xFF
	void PS2_EnterConfing(void);	         			//进入配置
	void PS2_TurnOnAnalogMode(void);         			//发送模拟量
	void PS2_VibrationMode(void);            			//振动设置
	void PS2_ExitConfing(void);	             			//完成配置
	void PS2_SetInit(void);		             			//配置初始化
	PSTwo_Data PS2_Receive (void);           			//获取手柄按键和杆量数据
	uint8_t PS2_check(void);                      		//检查手柄是否插入
	#endif

#ifdef __cplusplus
}
#endif


