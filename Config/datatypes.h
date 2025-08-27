#ifdef __cplusplus
extern "C" {
#endif
	
	#ifndef _DATATYPES_H_
	#define _DATATYPES_H_
	
	#include <stdint.h>
	#include <stdbool.h>
	#include <errno.h>
	#include <stdio.h>
	#include <stdarg.h>
	#include <string.h>
	#include <stdarg.h>
	#include <stdlib.h>
	#include <math.h>	
	#define PackNum(total,single)  (total+(single-1))/single
	#define PI      3.1415926
	#define ADC_Ratio 	124.1   //(4095*3K)/((3K+27K)*3.3V)
	#define ADC_Ratio_2 66.26   //(4095*2.2K)/((2.2K+39K)*3.3V)
	#define G_TO_ACCEL 9.81
	#define DEG_TO_RAD 0.01745329
	#define MGAUSS_TO_UTESLA 0.1
	#define UTESLA_TO_TESLA 0.000001

	#define GY85_ACCEL_SCALE 1 / 256 // LSB/g
	#define GY85_GYRO_SCALE 1 / 14.375 // LSB/(deg/s)
	#define GY85_MAG_SCALE 0.92 * MGAUSS_TO_UTESLA // uT/LSB

	//加速度计灵敏度 2^15/2g(加速度传感器量程值,±2,±4,±8,±16)
	//  32768/16 =2048
	#define MPU9250_ACCEL_SCALE 1 / 2048 // LSB/g
	//陀螺仪灵敏度 2^15/250g(加速度传感器量程值,±250,±500,±1000,±2000)
	//  32768/2000 =16.384
	#define MPU9250_GYRO_SCALE 1 / 16.384// LSB/(deg/s)

	#define MPU9250_MAG_SCALE 0.6 // uT/LSB

	#define MPU6050_ACCEL_SCALE 1 / 2048 // LSB/g
	#define MPU6050_GYRO_SCALE 1 / 16.384 // LSB/(deg/s)
	#define MPU6050_MAG_SCALE 0.285 // uT/LSB
	
	#define strBuffLength 150
	#define FRAME_HEAD_L          0xACu
	#define FRAME_HEAD_H          0x6Du  
	typedef enum 
	{
		SERIAL1 = 0x01,
		SERIAL2,
		SERIAL3,
		SERIAL4,
		SERIAL5,
		USB_SIO,
	}Serial_TypeDef; 	
	typedef enum	
	{   
		PACK_HEAD_L_INDEX      = 0x00 ,//包头低位位置
		PACK_HEAD_H_INDEX      		  ,//包头高位位置
		PACK_CMD_INDEX				  ,//指令位置
		RESULTS_TYPE_INDEX            ,//回复和指令类型位置
		PACK_LEN_L_INDEX			  ,//数据包长度低位位置
		PACK_LEN_H_INDEX			  ,//数据包长度高位位置
		PACK_DATA_INDEX			  	  ,//数据包起始位置
        RESULTS_OK              = 0x0A,//回复成功
        RESULTS_FAIL			      ,//回复失败
        CRC_ERROR			  		  ,//回复CRC校验失败
        PACK_HEAD_ERROR			      ,//回复包头错误
        PACK_CMD_ERROR			      ,//回复指令错误，下位机不支持该指令
        UI_CRC_ERROR			      ,//上层CRC校验错误
        UI_PACK_HEAD_ERROR			  ,//上层包头校验错误
        READ_TRPE                     ,//上位机读取指令类型,下位机回复指令类型
        SEND_TRPE                     ,//上位机发送指令类型
		UI_GET_CRC_ERROR			  ,//上位机读取APP 固件CRC参数错误
        CMD_WRITE_KEY          	= 0x3E,//密钥写入
        CMD_READ_CHECK                ,//校验信息读取
        CMD_READ_INFO      	 	= 0x40,//读取版本信息
        CMD_WRITE_INFO                ,//写入版本信息
        CMD_READ_DRIVE                ,//读取使用的驱动类型
        CMD_WRITE_DRIVE 		      ,//写入使用的驱动类型
        CMD_APPLY_FLAG_BIT            ,//电机和编码器标志位应用
        CMD_READ_FLAG_BIT             ,//电机和编码器标志位读取
        CMD_WRITE_FLAG_BIT            ,//电机和编码器标志位写入
        CMD_APPLY_PID                 ,//PID参数应用
        CMD_READ_PID      		      ,//PID参数读取
        CMD_WRITE_PID     		      ,//PID参数写入
        CMD_READ_VEL      		      ,//速度数据读取
        CMD_SET_VEL    		      	  ,//速度数据设置
        CMD_READ_EN      		      ,//外设使能标志位读取
        CMD_WRITE_EN    		      ,//外设使能标志位写入
        CMD_READ_PARAM      	      ,//参数读取
        CMD_WRITE_PARAM     	      ,//参数写入
        CMD_READ_BAT     		      ,//电压数据读取
        CMD_READ_SHOW     		      ,//电机调试信息读取
        CMD_READ_SONAR     		      ,//超声波数据读取
        CMD_SET_SERVO     	      	  ,//舵机角度设置
		CMD_LIN_VEL_CALIB             ,//线速度校准
        CMD_ANG_VEL_CALIB             ,//角速度校准
		CMD_ACTIVE_REPORT             ,//使能主动上报
		CMD_RESET_PAEAM               ,//恢复默认参数
		CMD_PORT_RELAID               ,//通信接口转发
		CMD_COM_INTER_SET             ,//通信接口参数配置
		CMD_COM_INTER_READ            ,//通信接口参数配置读取
		CMD_CPU_SOFT_RESET            ,//软件复位
		CMD_READ_FREQ     		      ,//读取发布频率
		CMD_SET_FREQ      		      ,//设置发布频率
		CMD_UPGRADER      		  	  ,//升级固件或引导
		CMD_READ_BOOT_VER             ,//读取BOOT版本
		CMD_READ_IMU_EN     		  ,//使能数据读取
		CMD_READ_IMU     		      ,//IMU数据读取
		CMD_READ_LIB_VER     		  ,//lib版本读取
		CMD_CUSTOM_PLOT_INIT          ,//自定义波形绘制初始化
		CMD_CUSTOM_PLOT_SEND          ,//自定义波形绘制数据发送
		CMD_TERMINAL_SEND			  ,//下位机发送数据到上位机终端界面
		CMD_TERMINAL_READ		  	  ,//上位机终端界面发送数据到下位机
		CMD_SET_MOVE_MODE			  ,//设置控制模式,解算模式\独立模式
		CMD_SET_MOVE_VALUE			  ,//设置独立模式各电机数据
		CMD_SET_MOTOR_ID			  ,//设置CAN电机驱动ID
		CMD_SET_SONAR                 ,//设置超声波参数
		CMD_GET_SONAR                 ,//读取超声波参数    
		CMD_SET_WIFI_PARAM            ,//设置wifi参数
		CMD_GET_WIFI_PARAM            ,//读取wifi参数
		CMD_GET_APP_CRC				  ,//读取APP CRC校验码
		CMD_ERASE_UP_CHECK			  ,//擦除升级校验信息
		CMD_APP_GO2_BOOT			  ,//APP跳转到Boot
		CMD_BOOT_GO2_APP			  ,//Boot跳转到APP
	} en_cmd_t;	
	typedef enum	
	{   
		RUN_NONE					=0,
		RUN_VEL						  ,
		RUN_RPM						  ,
		RUN_POINT					  ,
		RUN_LIN_VEL_CALIB             ,//线速度校准
		RUN_ANG_VEL_CALIB             ,//角速度校准
		RUN_PID_CALIB				  ,//PID校准
	} en_run_t;	
	typedef enum	
	{   
		RELAID_CAN					=0, 
		RELAID_MODBUS				  ,
		RELAID_RS232				  ,
		RELAID_BLE					  ,
		RELAID_UART4             	  ,
		RELAID_UART1				  ,
		RELAID_USB					  ,
		RELAID_ROS					  ,
		RESERVE1					  ,		//可修改名称
		RESERVE2					  ,		//可修改名称
		RESERVE3					  ,		//可修改名称
		RESERVE4					  ,		//可修改名称
		RELAID_NUM            	  	  ,		//可修改名称
	}relaid_type_t;	
	typedef enum	
	{   
		USB_Disable					=0,
		USB_Enabled					=1,
		RELAID_USBtoCAN				=2, 
		RELAID_USBtoUR3				  ,
		RELAID_USBtoUR5				  ,
		RELAID_USBtoUR2				  ,
		RELAID_USBtoUR4				  ,
		RELAID_USBtoUR1               ,
		RELAID_USBautoUR3             ,
		RELAID_USBautoUR2             ,
	}relaid_type_usb;
	typedef enum	
	{   
		UR2_Disable					=10,
		UR2_Enabled					=11,
		RELAID_UR2toCAN				=12, 
		RELAID_UR2toUR3				  ,
		RELAID_UR2toUR5			      ,
		RELAID_UR2toUR4				  ,
		RELAID_UR2toUR1               ,
		RELAID_UR2autoUR3             ,
		RELAID_UR2autoUR5             ,
	}relaid_type_ur2;	
	typedef enum	
	{   
		UR5_Disable					=20,
		UR5_Enabled					=21,
		RELAID_UR5toCAN				=22, 
		RELAID_UR5toUR3				  ,
		RELAID_UR5toUR2			      ,
		RELAID_UR5toUR4				  ,
		RELAID_UR5toUR1               ,
		RELAID_UR5autoUR3             ,
	}relaid_type_ur5;	
	typedef enum	
	{   
		UR1_Disable					=30,
		UR1_Enabled					=31,
		RELAID_UR1toCAN				=32, 
		RELAID_UR1toUR3				  ,
		RELAID_UR1toUR5			      ,
		RELAID_UR1toUR2			      ,
		RELAID_UR1toUR4				  ,
		RELAID_UR1autoUR3			  ,
		RELAID_UR1autoUR2			  ,
	}relaid_type_ur1;
	typedef enum	
	{   
		CAN_Disable					=40,
		CAN_Enabled					=41,
		RELAID_CANtoUR3				=42, 
		RELAID_CANtoUR5				  ,
		RELAID_CANtoUR2			      ,
		RELAID_CANtoUR4			      ,
		RELAID_CANtoUR1				  ,
	}relaid_type_can;
	typedef enum	
	{   
		RELAID_UR1toUR1				=50,	//不可修改名称
		RELAID_UR2toUR2				=51,	//不可修改名称
		RELAID_UR3toUR3				=52, 	//不可修改名称
		RELAID_UR4toUR4				  ,		//不可修改名称
		RELAID_UR5toUR5			      ,		//不可修改名称
		RELAID_CANtoCAN               ,		//不可修改名称
		RELAID_XXXtoROS				  ,		//不可修改名称
		RELAID_RESERVE1				  ,		//可修改名称
		RELAID_RESERVE2				  ,		//可修改名称
		RELAID_RESERVE3				  ,		//可修改名称
		RELAID_RESERVE4						//可修改名称
	}relaid_type_send;
	typedef enum	
	{   
		ROBOT_D2	= 0x01,
		ROBOT_D4		  ,
		ROBOT_A1		  ,
		ROBOT_A2		  ,
		ROBOT_O3		  ,
		ROBOT_O4		  ,
		ROBOT_M4			
	} robot_dev_t;
	typedef enum	
	{   
		M_A4950		= 0x01,
		M_BTN797X	  	  ,
		M_ESC		  	  ,
		M_ESC_ENC		  ,
		M_DCM_4WD		  ,
		M_CAN_1_2		  ,
		M_CAN_1_1		  ,
		M_CAN_ENC		  ,
	}motor_drive_t;	
	typedef enum	
	{   
		I_GY85		= 0x01,
		I_MPU9250_N	  	  ,
		I_MPU9250_W		  ,
		I_MPU6050_N		  ,
		I_MPU6050_W		  ,
		I_MPU6500_N		  ,
		I_MPU6500_W		  ,
	}imu_drive_t;
	typedef enum	
	{   
		PACKET_ACK_OK               = 0x00, //正确回复
        PACKET_ACK_ERROR            = 0x01, //错误回复
        PACKET_ACK_ABORT            = 0x02, //终止
        PACKET_ACK_TIMEOUT          = 0x03, //超时
        PACKET_ACK_ADDR_ERROR       = 0x04, //地址错误
        PACKET_ACK_FLASH_SIZE_ERROR = 0x05, //FALASH大小错误
        PACKET_ACK_CRC_ERROR		= 0x06, //写入文件CRC校验错误
        PACKET_ACK_DEVICE_M_ERROR   = 0x07, //设备型号错误
        PACKET_ACK_HARDWARE_V_ERROR = 0x08, //硬件版本错误
        PACKET_ACK_SOFTWARE_V_ERROR = 0x09, //软件版本错误
        PACKET_ACK_READ_SN_ERROR    = 0x0A, //写入设备SN错误
        PACKET_ACK_WRITE_SN_ERROR   = 0x0B, //读取设备SN错误
		PACKET_ACK_GoToApp_ERROR	= 0x0C,	//拷贝固件失败，拷贝后的CRC错误
//		 PACKET_CMD_HANDSHAKE    = 0xA1,//通信握手
//       PACKET_CMD_APP_DOWNLOAD       ,//下载应用程序数据
//       PACKET_CMD_BOOT_DOWNLOAD      ,//下载引导程序数据
//       PACKET_CMD_AGAIN              ,//失败重发
//       PACKET_CMD_FLASH_CRC          ,//计算flash校验值
		UP_HANDSHAKE			 =0xA1, //通信握手
		UP_WRITE_APP				  , //下载应用程序数据
		UP_WRITE_BOOT				  , //下载应用程序数据
		UP_AGAIN				      , //失败重发
		UP_APP_FLASH_CRC      		  ,//计算APP固件flash校验值
		UP_BOOT_FLASH_CRC     		  ,//计算BOOT固件flash校验值
		UP_BOOT_GOTO_APP			  ,//BOOT跳转到APP
		UP_NUM             	 	 =0xFF,
		UP_CMD_GOTO_APP          =0x8F,	//没有升级请求，直接跳转到APP
		UP_CMD_COPY_APP          =0x9F,	//有升级请求，需要把升级备份区代码拷贝到APP
		UP_CMD_COPY_BOOT         =0xAF, //有升级请求，需要把升级备份区代码拷贝到Boot
		UP_APP_GO_BOOT         	 =0xBF, //APP跳转到BOOT
//		RPR_APPTOBOOT			 =0xFE,	//APP下载固件完成,CRC校验成功;需要设置升级请求标志位并跳转到BOOT
//		RPR_BOOTCOPYTOAPP		 =0xFD,	//BOOT下载固件完成,CRC校验成功;需要拷贝固件到APP并跳转到APP
	}upgrader_type_t;
	typedef enum 
    {
		PACKET_CMD_INDEX     = PACK_DATA_INDEX	 ,//升级指令位置              	6
		PACKET_RESULT_INDEX           			 ,//升级包状态回复位置			7
		PACKET_RDATA_INDEX            			 ,//升级数据回复数据起始位置	8
		PACKET_FRAME_L_INDEX =PACKET_CMD_INDEX+1 ,//升级数据帧低位				7
		PACKET_FRAME_H_INDEX					 ,//升级数据帧低位				8
		PACKET_FIR_DATA_INDEX					 ,//升级数据位置				9
    }en_packet_index_t;							  //升级包指令集位置
	typedef struct upGraderDate_t
	{	
		uint8_t  upCmd;							//升级指令
		uint16_t frameNum;						//升级帧
		uint16_t frameNumOld;					//升级历史帧
		uint16_t fileCrc;						//升级固件CRC
		uint32_t fileSize;						//升级固件大小
		uint32_t wFirSizeIndex;					//升级写入大小位置
		uint32_t wFirSize;						//升级写入大小
		uint32_t wFirAddr;						//升级写入地址
		uint8_t  wFirBuf[1024];
	}upGraderDate;

	typedef struct _stcSonarCfg_
	{
		uint8_t En1;							//1 脉冲方式 2 Modbus
		uint8_t En2;							//1 脉冲方式 2 Modbus
		uint8_t En3;							//1 脉冲方式 2 Modbus
		uint8_t En4;							//1 脉冲方式 2 Modbus
		uint16_t DeceDist1;
		uint16_t DeceDist2;
		uint16_t DeceDist3;
		uint16_t DeceDist4;
		uint16_t EstopDist1;
		uint16_t EstopDist2;
		uint16_t EstopDist3;
		uint16_t EstopDist4;
	}stcSonarCfg;
	typedef struct upGraderFlash_t
	{	
		//0x8F 0xFF:没有更新操作由BOOT跳转到APPP 0x9F:(App/Boot)下载App固件完成，CRC校验成功，拷贝固件并重启(App使用重启，Boot使用跳转)
		//0xAF:App下载Boot固件完成，CRC校验成功，拷贝固件并重启
		uint16_t upGoCmd;		
		uint16_t fileCrc;
		uint32_t fileSize;
		uint32_t backupsAddr;	//备份区地址
	}upGraderFlash;
	typedef struct
	{
		uint8_t  BRP_t;
		uint8_t  RSAW_t;
		uint8_t  BTS1_t;
		uint8_t  BTS2_t;
		uint16_t BaudRate;
		uint32_t APB_Clk;
	}canConfig;
	typedef struct _DirectionMark_
	{
		int Motor;
		int Encoder;
		uint32_t Id;
	}DirectionMark;
	typedef struct _Hardware_Struct_
	{
		uint8_t Hardware_Ver[17];
		uint8_t Software_Ver[17];
		uint8_t Device_SN[17];
		uint8_t ActStatus;
		uint32_t DevNumber;
		uint32_t FlashSize;
	}Hardware_Struct;
	typedef struct wifiParam_t
	{	
		uint8_t  len;							//升级指令
		uint8_t  data[31];
	}wifiParam;
	typedef struct _pidParam_
	{
		float K_p;
		float K_i;
		float K_d;
	}pidParam;
	typedef struct _peripheralsEn_
    {
        uint8_t servo1;
        uint8_t servo2;
        uint8_t servo3;
        uint8_t servo4;
        uint8_t rgbled;
        uint8_t lowBat;
    }peripheralsEn;
	typedef struct _appInterfaceParam_
	{
	  uint32_t Port_BaudRate;   //波特率
	  uint8_t  Port_WordLength;	//数据位
	  uint8_t  Port_StopBits;	//停止位
	  uint8_t  Port_Parity;     //校验位
	  uint8_t  Port_Mode;       //模式	
	  uint8_t  Port_HFC ;		//流控	  
	}appInterfaceParam;
	typedef struct _Param_ 
	{
		uint8_t RobotType;
		uint8_t MotorType;
		uint8_t IMUType;
		uint8_t ROS_SERIALx;
		uint8_t ROSType;
		uint8_t MotorDebug;
		uint8_t ControlMode;
		uint8_t InfoEnabled;
		peripheralsEn perEnabled;
		uint8_t WS2812Enabled;
		uint8_t PubVel_Hz;
		uint8_t PubImu_Hz;
		uint8_t PubBat_Hz;
		uint8_t PubSonar_Hz;
		uint8_t TurnInitAngle;
		uint8_t MaxSteerAngle;
		uint16_t EscMedianValue;
		uint16_t Max_RPM;
		uint16_t Max_PWM;
		uint16_t Pwm_psc;
		uint16_t Pwm_arr;
		
		float WheelDiameter;
		float LRInterval;
		float FBInterval;
		float CountsPer;
		float Max_Speed;
		float RobotRadius;

		pidParam p_M1;
		pidParam p_M2;
		pidParam p_M3;
		pidParam p_M4;
		DirectionMark M1;
		DirectionMark M2;
		DirectionMark M3;
		DirectionMark M4;

		appInterfaceParam uart1Cfg;
		appInterfaceParam uart2Cfg;
		appInterfaceParam uart3Cfg;
		appInterfaceParam uart4Cfg;
		appInterfaceParam uart5Cfg;
		canConfig canSetParam;
		stcSonarCfg sonarCfg;
		
		// 0： USB不开启主动上报    1：USB开启主动上报    2：USB-CAN转发    3：USB-RS485转发    4：USB-RS232转发 	 5：USB-BLE转发     6：USB-UART4转发 	7：USB-UART1转发    8:USB-RS485透明转发     9:USB-BLE透明转发
		// 10：BLE不开启主动上报   11：BLE开启主动上报   12：BLE-CAN转发   13：BLE-RS485转发   14：BLE-RS232转发 	15：NULL           16：BLE-UART4转发   17：BLE-UART1转发   18:BLE-RS485透明转发    19:BLE-RS232透明转发
		// 20：RS232不开启主动上报 21：RS232开启主动上报 22：RS232-CAN转发 23：RS232-RS485转发 24：NULL 			25：RS232-BLE转发  26：RS232-UART4转发 27：RS232-UART1转发 28:RS232-RS485透明转发  29:NULL
		// 30：UART1不开启主动上报 31：UART1开启主动上报 32：UART1-CAN转发 33：UART1-RS485转发 34：UART1-RS232转发  35：UART1-BLE转发  36：UART1-UART4转发 37: NULL			   38:UART1-RS485透明转发  39:UART1-BLE透明转发
		uint8_t IsUsbLink;      
		uint8_t IsRosNodePub;		//0:ROS没有连接,1:ROS连接成功,2:IMU可视化数据上报使能,3:ROS数据转发
		
		uint8_t wifiNum;
		wifiParam wifiData[10];
		
		uint8_t cksum;
		uint8_t FirstWrite;
	}Param_Struct;

	struct imu_data_s
	{
		  short x;
		  short y;
		  short z;
	};
	struct imu_check
	{
		  uint8_t acc;
		  uint8_t gyro;
		  uint8_t magn;
	};
	typedef struct _Moto_
	{
		float Current_Rpm1;
		float Current_Rpm2;
		float Current_Rpm3;
		float Current_Rpm4;
		float MotorEncoder1;
		float MotorEncoder2;
		float MotorEncoder3;
		float MotorEncoder4;
	}_Moto_Str;
	typedef struct SonarDate_t
	{
		float Sonar1;
		float Sonar2;
		float Sonar3;
		float Sonar4;
	}SonarDate;
	typedef struct _DeviceEFP_
	{
		int Pwm_Out;
		int Expectations;
		int Feedback;
	}DeviceEFP;
	typedef struct
	{	
		uint8_t     length_t;
		uint8_t     DataBuff[strBuffLength];
	}stcATBuff;
	typedef struct
	{	
		float linearVelX;
		float linearVelY;
		float angularVelZ;
	}stcTwist;
	typedef struct
	{	
		float VelX;
		float VelY;
		float VelZ;
		uint8_t isLock;
	}stcMoveVel;
#pragma pack(push)  //保存对齐
#pragma pack(1) 	//设定为1字节对齐
	typedef struct
	{	
		uint8_t mode;
		uint8_t loop;
		float setValue;
		float setValue2;
	}setIndepend;
	typedef struct
	{	
		uint8_t m_EnableName;
		setIndepend m1_Value;
		setIndepend m2_Value;
		setIndepend m3_Value;
		setIndepend m4_Value;
	}stcIndepend;
	typedef struct _gps_report_
	{
		uint32_t tim_pos; 				//< Timestamp for position information 位置信息的时间戳*/
		uint16_t year;					//年
		uint8_t  month;					//月
		uint8_t  day;					//日
		uint8_t  hour;					//时
		uint8_t  min;					//分
		uint8_t  sec;					//秒
		int32_t lat;                 	/**< Latitude in 1E-7 degrees  纬度1E-7度*/
		int32_t lon;                 	/**< Longitude in 1E-7 degrees 经度1E-7度*/
		int32_t alt;                 	/**< Altitude in 1E-3 meters (millimeters) above MSL  海拔高度，高于MSL 1E-3米（毫米）*/
	} gps_report;
#pragma pack(pop)   //恢复对齐状态
	typedef struct
	{	
		DeviceEFP M1;
		DeviceEFP M2;
		DeviceEFP M3;
		DeviceEFP M4;
	}stcMotorDebug;
	typedef struct
	{	
		float Vel_x;
		float Vel_y;
		float Vel_z;
		float EncM1;
		float EncM2;
		float EncM3;
		float EncM4;
	}stcMotorVel;
	typedef struct _vMotorStr_
	{	
		int dutyVal;
		int mDir;
		uint8_t mName;
		uint8_t mDirveType;
		uint32_t mId;
	}vMotorStr;
	typedef struct
	{
	  uint32_t StdId;  /*!< Specifies the standard identifier.This parameter can be a value between 0 to 0x7FF. */
	  uint32_t ExtId;  /*!< Specifies the extended identifier.This parameter can be a value between 0 to 0x1FFFFFFF. */
	  uint8_t IDE;     /*!< Specifies the type of identifier for the message that 
							will be transmitted. This parameter can be a value of  CAN_Id_Extended:0x04 CAN_Id_Standard:0x00*/
	  uint8_t RTR;     /*!< Specifies the type of frame for the message that will 
							be transmitted. This parameter can be a value of  CAN_RTR_Remote:0x02 CAN_RTR_Data:0x00 */
	  uint8_t DLC;     /*!< Specifies the length of the frame that will be 
							transmitted. This parameter can be a value between 0 to 8 */
	  uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0 to 0xFF. */
	  uint8_t FMI;     /*!< Specifies the index of the filter the message stored in  the mailbox passes through. This parameter can be a value between 0 to 0xFF */
	} canStrMsg;
	
	typedef struct
	{
		stcATBuff Value;
		uint32_t DevId;
	} rosRelaidrMsg;
	typedef struct _dutyStruct_ 
	{
		uint8_t  CaptuerStatus[2];
		uint16_t dutyStart[2];
		uint16_t dutyValue[2];
	}dutyStruct;
	
	typedef struct _ppmStruct_ 
	{
		uint8_t  ppmChId;
		uint16_t ppmCapStart;
		uint16_t ppmCapEnd;
		uint16_t ppmValue[10];
	}ppmStruct;
	
	typedef uint8_t(*comInterfa_CallBack) (void *payload);
	typedef struct _ComSendFunction {
		comInterfa_CallBack callback;
	} ComSendFunction;
	
	
	extern Param_Struct configParam;
	#endif
#ifdef __cplusplus
}
#endif

















