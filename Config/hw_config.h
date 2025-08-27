
#ifndef _HW_CONFIG_H_
#define _HW_CONFIG_H_
	#ifdef __cplusplus
	extern "C" {
	#endif
	#include "stdint.h"
	#include "datatypes.h"
	#define BOOTSOF_VER             "Bootloader-V1.9"
	typedef enum {
		MOTOR1 = 0, //left back
		MOTOR2 = 1, //right back
		MOTOR3 = 2, //right front
		MOTOR4 = 3, //left front
		MOTOR_END = 4
	}Motor_TypeDef; 
	typedef enum {
		Motor_PID1 = 0,
		Motor_PID2 = 1,
		Motor_PID3 = 2,
		Motor_PID4 = 3,
		Motor_PID_END = 4
	}PID_TypeDef; 
	typedef enum {
		ENCODER1 = 0,
		ENCODER2 = 1,
		ENCODER3 = 2,
		ENCODER4 = 3,
		ENCODER_END = 4
	}Encoder_TypeDef; 
	typedef enum {
		LED_STATUS = 0,
		LED_RUN = 1,
		LED_FAULT = 2,
		LED_END = 3
	}LED_TypeDef; 
	typedef enum {
		Beep = 0,
		Beep_END = 1
	}Beep_TypeDef; 
	typedef enum {
		temp_Mcu = 0,
		voltage_Bus ,
		current_M1 	,
		current_M2 	,
		current_M3 	,
		current_M4	,
		temp_M1		,
		temp_M2 	,
		M_ADC_END	,
		ADCRn      	,
	}M_ADC_TypeDef; 
	typedef enum {
		Sonar1 = 0,
		Sonar2 = 1,
		Sonar_END = 2
	}Sonar_TypeDef; 
	typedef enum {
		SERVO1 = 0,
		SERVO2 = 1,
		SERVO3 = 2,
		SERVO4 = 3,
		SERVO_END = 4
	}Servo_TypeDef;
	typedef enum {
		IMU_N = 0,
		IMU_W = 1,
		OLED_S = 2,
		IIC_END = 3
	}IIC_TypeDef; 
	typedef enum {
		POWER_KEY1 = 0,
		POWER_KEY2 = 1,
		POWER_KEY3 = 2,
		POWER_KEY_END = 3
	}POWER_KEY_TypeDef; 
	typedef enum {
		NONE_Er				= 0x00,
		ACTIVATE_NO_Er 		= 0x01,			//设备未激活报警 		6
		POWER_LOW_Er   		= 0x02,			//低压报警            	8
		IMU_CHECK_Er    	= 0x04,			//IMU校验失败报警		10
		MCU_TEP_OVER_Er 	= 0x08,			//MCU温度超过60度报警	12
		SAVE_PARAM_Er		= 0x10,			//参数保存失败报警		14
		ESTOPIC_Er			= 0x20,			//急停报警				16
	}ERROR_TypeDef; 
	typedef enum {
		WS2812_1 = 0,
		WS2812_2 = 1,
		WS2812_END = 2
	}WS2812_TypeDef;
	typedef enum {
		activate_Pri		= 2	,	//2
		timeout_Pri			= 2	,	//2
		LedBeep_Pri 		= 3	,	//3
		ErrorManage_Pri		= 3	,	//3
		terminal_Pri			,   //4
		Ws2812Run_Pri			,	//5
		OledShow_Pri			,	//6
		decManage_Pri			,	//7
		MoveBase_Pri			,	//8
		BatPub_Pri				,	//8
		SonaroPub_Pri			,	//9
		BaseInfoPub_Pri 		,	//10
		ImuPub_Pri				,	//
		VelPub_Pri				,	//
		msgsSend_Pri			,	//
		uart5Rx_Pri				,	//
		uart5Tx_Pri				,	//15
		uart3Rx_Pri				,	//
		uart3Tx_Pri				,	//
		uart4Rx_Pri				,	//
		uart4Tx_Pri				,	//
		hcBleRx_Pri				,	//20
		hcBleTx_Pri				,	//
		canLinkTx_Pri			,	//
		canlinkRx_Pri			,	//
		RosSub_Pri              ,   //
		usbLinkTx_Pri			,	//25
		usblinkRx_Pri			,	//
		uart1Rx_Pri				,	//
		uart1Tx_Pri				,	//
		configParam_Pri			,	//
		ros_task_Pri			,	//30
	}TaskPri_TypeDef;
	void BaseBoard_TIM6_Init();
	void SoftReset_fun(void);	
	void hw_GPIO_Init(void);
	void TIM_SetCompareX(void *tmr_x, uint16_t tmr_channel,uint32_t tmr_channel_value);
	void MotorInit(uint8_t mName_t,uint32_t mPwm_Arr,uint32_t mPwm_Psc,uint8_t mMotorType);
	void EncoderInit(uint8_t eName_t,uint8_t mMotorType);
	void ServoInit(uint8_t sName_t,uint32_t mPwm_Arr,uint32_t mPwm_Psc);
	void getUARTConfigParam(const appInterfaceParam *inData,void *outPut);
	void Sonar1_IRQHandler(void);															//TIM11IRQ
	void Sonar2_IRQHandler(void);
	void AppSioSendProcessing(stcATBuff *buff_t);
	void AppSioSendProcessingProt(stcATBuff *buff_t,uint8_t SioName);
	void Ws2812TimeInit(uint8_t LedType);
	void Ws2812SendBuffDma(uint8_t LedType,uint16_t *sendBuff,uint16_t buffLen);
	void TIM_MotorSpin(uint8_t mName_t,int Dir_t,uint32_t Duty_t);
	void setEStopStatus(uint8_t status_t);
	void setMoveVelData(const stcMoveVel *src_t);
	void dec_dutyInit(void);
	void dec_ppmInit(void);
	uint32_t getSysTickCnt(void);
	SonarDate getUartSonarValue(void);
	#define WATCHDOG_RESET_MS 	150	/*看门狗复位时间*/
	void watchdogInit(uint16_t xms);
	#define BOOT_ADDR 		0x8000000	//0~32K
	#define APP_ADDR 		0x8010000	//64~256K
	#define APP_CHECH 		0x8010420	//APP_ADDR+0x420
	#define BOOT_SIZE 		0x8000		//32K
	#define APP_PARAM_ADDR  0x8008000  	//32~48k
	#define UP_PARAM_ADDR  	0x8004000  	//48~64k
	#define UPGRADE_ADDR	0x8040000  	//256k
	#define UPGRADE_CHECH 	0x8040420	//UPGRADE_ADDR+0x420
	#define f64K_VALUE		0x10000		//64K
	#define f192K_VALUE		0x30000		//192K
	#define f320K_VALUE		0x50000		//320K
	#define SOFTWARE_VER    "FreeRTOS-V1.6.5"
	
	
	/* Define the size of the sectors to be used */
	#define PAGE_SIZE             (uint32_t)0x4000  /* Page size = 16KByte 模拟 flash 大小16K */
	/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
	be done by word  */
	#define VOLTAGE_RANGE         (uint8_t)VoltageRange_3
	/* EEPROM start address in Flash */
	#define EEPROM_START_ADDRESS  ((uint32_t)APP_PARAM_ADDR) 	/* EEPROM emulation start address:\
																 from sector1 : after 16KByte of used Flash memory */
	/* Pages 0 and 1 base and end addresses */
	#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
	#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))
	#define PAGE0_ID              FLASH_Sector_2

	#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE))
	#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))
	#define PAGE1_ID              FLASH_Sector_3
	
	
//	#define ST_HV126					//1.26硬件则启用该宏
	#define NB_OF_VAR       ((uint16_t)((2 * sizeof(upGraderFlash) + sizeof(Param_Struct) + 1) / 2))
	#ifdef STM32F40_41xxx
		#ifdef ST_HV126
			#include "hw_STM32F40x_126.h"
		#else 
			#include "hw_STM32F40x_127.h"
		#endif
		#define ACTIVATE_ADDR 		0x1FFF7800
		#define FALSH_SIZE_ADDR 	((uint32_t*)0x1FFF7A22)
		#define FLASH_SECTOR_SIZE 	0x800
		#define CAN_APBH_CLK		42000					
		#define mcuName 			1 			
		#define watchdogReset() 	(IWDG_ReloadCounter())
	#elif  AT32F40x
		#include "hw_AT32F40x.h"
		#define ACTIVATE_ADDR 		0x1FFFF7E0
		#define FALSH_SIZE_ADDR 	((uint32_t*)0x1FFFF7E0)
		#define FLASH_SECTOR_SIZE 	0x800
		#define CAN_APBH_CLK		120000
		#define mcuName 			2 
		#define watchdogReset() 	(wwdt_reset())
	#elif STM32F10X_HD
		#include "hw_STM32F10x.h"
		#define ACTIVATE_ADDR 		0x1FFFF7E0
		#define FALSH_SIZE_ADDR 	((uint32_t*)0x1FFFF7E0)
		#define FLASH_SECTOR_SIZE 	0x800
		#define CAN_APBH_CLK		120000
		#define mcuName 			3 
		#define watchdogReset() 	(IWDG_ReloadCounter())
	#endif
	
	#ifndef BOOTLOADER
		#define SYSTEM_SUPPORT_OS   1
	#endif

	//输出日志级别
	#define ELOG_LVL_ASSERT                      0						//断言
	#define ELOG_LVL_ERROR                       1						//错误
	#define ELOG_LVL_WARN                        2						//警告
	#define ELOG_LVL_INFO                        3						//信息
	#define ELOG_LVL_DEBUG                       4						//调试
	#define ELOG_LVL_VERBOSE                     5						//详细的
	#define ELOG_OUTPUT_LVL                      ELOG_LVL_VERBOSE
	//输出使能
	#if ELOG_OUTPUT_LVL >= ELOG_LVL_ASSERT
		#define elog_assert(tag, ...) \
				elog_output(ELOG_LVL_ASSERT, tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
	#endif
	#if ELOG_OUTPUT_LVL >= ELOG_LVL_ERROR
		#define elog_error(tag, ...) \
				elog_output(ELOG_LVL_ERROR, tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
	#endif

	#if ELOG_OUTPUT_LVL >= ELOG_LVL_WARN
		#define elog_warn(tag, ...) \
				elog_output(ELOG_LVL_WARN, tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
	#endif

	#if ELOG_OUTPUT_LVL >= ELOG_LVL_INFO
		#define elog_info(tag, ...) \
				elog_output(ELOG_LVL_INFO, tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
	#endif

	#if ELOG_OUTPUT_LVL >= ELOG_LVL_DEBUG
		#define elog_debug(tag, ...) \
				elog_output(ELOG_LVL_DEBUG, tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
	#endif

	#if ELOG_OUTPUT_LVL == ELOG_LVL_VERBOSE
		#define elog_verbose(tag, ...) \
				elog_output(ELOG_LVL_VERBOSE, tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
	#endif
	
	#if !defined(assert)
		#define assert           ELOG_ASSERT
	#endif
	void elog_output(uint8_t level, const char *tag, const char *file, const char *func,
		const long line, const char *format, ...);

	#ifdef __cplusplus
	}
	#endif	
#endif




