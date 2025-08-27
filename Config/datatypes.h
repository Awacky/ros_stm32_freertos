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

	//���ٶȼ������� 2^15/2g(���ٶȴ���������ֵ,��2,��4,��8,��16)
	//  32768/16 =2048
	#define MPU9250_ACCEL_SCALE 1 / 2048 // LSB/g
	//������������ 2^15/250g(���ٶȴ���������ֵ,��250,��500,��1000,��2000)
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
		PACK_HEAD_L_INDEX      = 0x00 ,//��ͷ��λλ��
		PACK_HEAD_H_INDEX      		  ,//��ͷ��λλ��
		PACK_CMD_INDEX				  ,//ָ��λ��
		RESULTS_TYPE_INDEX            ,//�ظ���ָ������λ��
		PACK_LEN_L_INDEX			  ,//���ݰ����ȵ�λλ��
		PACK_LEN_H_INDEX			  ,//���ݰ����ȸ�λλ��
		PACK_DATA_INDEX			  	  ,//���ݰ���ʼλ��
        RESULTS_OK              = 0x0A,//�ظ��ɹ�
        RESULTS_FAIL			      ,//�ظ�ʧ��
        CRC_ERROR			  		  ,//�ظ�CRCУ��ʧ��
        PACK_HEAD_ERROR			      ,//�ظ���ͷ����
        PACK_CMD_ERROR			      ,//�ظ�ָ�������λ����֧�ָ�ָ��
        UI_CRC_ERROR			      ,//�ϲ�CRCУ�����
        UI_PACK_HEAD_ERROR			  ,//�ϲ��ͷУ�����
        READ_TRPE                     ,//��λ����ȡָ������,��λ���ظ�ָ������
        SEND_TRPE                     ,//��λ������ָ������
		UI_GET_CRC_ERROR			  ,//��λ����ȡAPP �̼�CRC��������
        CMD_WRITE_KEY          	= 0x3E,//��Կд��
        CMD_READ_CHECK                ,//У����Ϣ��ȡ
        CMD_READ_INFO      	 	= 0x40,//��ȡ�汾��Ϣ
        CMD_WRITE_INFO                ,//д��汾��Ϣ
        CMD_READ_DRIVE                ,//��ȡʹ�õ���������
        CMD_WRITE_DRIVE 		      ,//д��ʹ�õ���������
        CMD_APPLY_FLAG_BIT            ,//����ͱ�������־λӦ��
        CMD_READ_FLAG_BIT             ,//����ͱ�������־λ��ȡ
        CMD_WRITE_FLAG_BIT            ,//����ͱ�������־λд��
        CMD_APPLY_PID                 ,//PID����Ӧ��
        CMD_READ_PID      		      ,//PID������ȡ
        CMD_WRITE_PID     		      ,//PID����д��
        CMD_READ_VEL      		      ,//�ٶ����ݶ�ȡ
        CMD_SET_VEL    		      	  ,//�ٶ���������
        CMD_READ_EN      		      ,//����ʹ�ܱ�־λ��ȡ
        CMD_WRITE_EN    		      ,//����ʹ�ܱ�־λд��
        CMD_READ_PARAM      	      ,//������ȡ
        CMD_WRITE_PARAM     	      ,//����д��
        CMD_READ_BAT     		      ,//��ѹ���ݶ�ȡ
        CMD_READ_SHOW     		      ,//���������Ϣ��ȡ
        CMD_READ_SONAR     		      ,//���������ݶ�ȡ
        CMD_SET_SERVO     	      	  ,//����Ƕ�����
		CMD_LIN_VEL_CALIB             ,//���ٶ�У׼
        CMD_ANG_VEL_CALIB             ,//���ٶ�У׼
		CMD_ACTIVE_REPORT             ,//ʹ�������ϱ�
		CMD_RESET_PAEAM               ,//�ָ�Ĭ�ϲ���
		CMD_PORT_RELAID               ,//ͨ�Žӿ�ת��
		CMD_COM_INTER_SET             ,//ͨ�Žӿڲ�������
		CMD_COM_INTER_READ            ,//ͨ�Žӿڲ������ö�ȡ
		CMD_CPU_SOFT_RESET            ,//�����λ
		CMD_READ_FREQ     		      ,//��ȡ����Ƶ��
		CMD_SET_FREQ      		      ,//���÷���Ƶ��
		CMD_UPGRADER      		  	  ,//�����̼�������
		CMD_READ_BOOT_VER             ,//��ȡBOOT�汾
		CMD_READ_IMU_EN     		  ,//ʹ�����ݶ�ȡ
		CMD_READ_IMU     		      ,//IMU���ݶ�ȡ
		CMD_READ_LIB_VER     		  ,//lib�汾��ȡ
		CMD_CUSTOM_PLOT_INIT          ,//�Զ��岨�λ��Ƴ�ʼ��
		CMD_CUSTOM_PLOT_SEND          ,//�Զ��岨�λ������ݷ���
		CMD_TERMINAL_SEND			  ,//��λ���������ݵ���λ���ն˽���
		CMD_TERMINAL_READ		  	  ,//��λ���ն˽��淢�����ݵ���λ��
		CMD_SET_MOVE_MODE			  ,//���ÿ���ģʽ,����ģʽ\����ģʽ
		CMD_SET_MOVE_VALUE			  ,//���ö���ģʽ���������
		CMD_SET_MOTOR_ID			  ,//����CAN�������ID
		CMD_SET_SONAR                 ,//���ó���������
		CMD_GET_SONAR                 ,//��ȡ����������    
		CMD_SET_WIFI_PARAM            ,//����wifi����
		CMD_GET_WIFI_PARAM            ,//��ȡwifi����
		CMD_GET_APP_CRC				  ,//��ȡAPP CRCУ����
		CMD_ERASE_UP_CHECK			  ,//��������У����Ϣ
		CMD_APP_GO2_BOOT			  ,//APP��ת��Boot
		CMD_BOOT_GO2_APP			  ,//Boot��ת��APP
	} en_cmd_t;	
	typedef enum	
	{   
		RUN_NONE					=0,
		RUN_VEL						  ,
		RUN_RPM						  ,
		RUN_POINT					  ,
		RUN_LIN_VEL_CALIB             ,//���ٶ�У׼
		RUN_ANG_VEL_CALIB             ,//���ٶ�У׼
		RUN_PID_CALIB				  ,//PIDУ׼
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
		RESERVE1					  ,		//���޸�����
		RESERVE2					  ,		//���޸�����
		RESERVE3					  ,		//���޸�����
		RESERVE4					  ,		//���޸�����
		RELAID_NUM            	  	  ,		//���޸�����
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
		RELAID_UR1toUR1				=50,	//�����޸�����
		RELAID_UR2toUR2				=51,	//�����޸�����
		RELAID_UR3toUR3				=52, 	//�����޸�����
		RELAID_UR4toUR4				  ,		//�����޸�����
		RELAID_UR5toUR5			      ,		//�����޸�����
		RELAID_CANtoCAN               ,		//�����޸�����
		RELAID_XXXtoROS				  ,		//�����޸�����
		RELAID_RESERVE1				  ,		//���޸�����
		RELAID_RESERVE2				  ,		//���޸�����
		RELAID_RESERVE3				  ,		//���޸�����
		RELAID_RESERVE4						//���޸�����
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
		PACKET_ACK_OK               = 0x00, //��ȷ�ظ�
        PACKET_ACK_ERROR            = 0x01, //����ظ�
        PACKET_ACK_ABORT            = 0x02, //��ֹ
        PACKET_ACK_TIMEOUT          = 0x03, //��ʱ
        PACKET_ACK_ADDR_ERROR       = 0x04, //��ַ����
        PACKET_ACK_FLASH_SIZE_ERROR = 0x05, //FALASH��С����
        PACKET_ACK_CRC_ERROR		= 0x06, //д���ļ�CRCУ�����
        PACKET_ACK_DEVICE_M_ERROR   = 0x07, //�豸�ͺŴ���
        PACKET_ACK_HARDWARE_V_ERROR = 0x08, //Ӳ���汾����
        PACKET_ACK_SOFTWARE_V_ERROR = 0x09, //����汾����
        PACKET_ACK_READ_SN_ERROR    = 0x0A, //д���豸SN����
        PACKET_ACK_WRITE_SN_ERROR   = 0x0B, //��ȡ�豸SN����
		PACKET_ACK_GoToApp_ERROR	= 0x0C,	//�����̼�ʧ�ܣ��������CRC����
//		 PACKET_CMD_HANDSHAKE    = 0xA1,//ͨ������
//       PACKET_CMD_APP_DOWNLOAD       ,//����Ӧ�ó�������
//       PACKET_CMD_BOOT_DOWNLOAD      ,//����������������
//       PACKET_CMD_AGAIN              ,//ʧ���ط�
//       PACKET_CMD_FLASH_CRC          ,//����flashУ��ֵ
		UP_HANDSHAKE			 =0xA1, //ͨ������
		UP_WRITE_APP				  , //����Ӧ�ó�������
		UP_WRITE_BOOT				  , //����Ӧ�ó�������
		UP_AGAIN				      , //ʧ���ط�
		UP_APP_FLASH_CRC      		  ,//����APP�̼�flashУ��ֵ
		UP_BOOT_FLASH_CRC     		  ,//����BOOT�̼�flashУ��ֵ
		UP_BOOT_GOTO_APP			  ,//BOOT��ת��APP
		UP_NUM             	 	 =0xFF,
		UP_CMD_GOTO_APP          =0x8F,	//û����������ֱ����ת��APP
		UP_CMD_COPY_APP          =0x9F,	//������������Ҫ���������������뿽����APP
		UP_CMD_COPY_BOOT         =0xAF, //������������Ҫ���������������뿽����Boot
		UP_APP_GO_BOOT         	 =0xBF, //APP��ת��BOOT
//		RPR_APPTOBOOT			 =0xFE,	//APP���ع̼����,CRCУ��ɹ�;��Ҫ�������������־λ����ת��BOOT
//		RPR_BOOTCOPYTOAPP		 =0xFD,	//BOOT���ع̼����,CRCУ��ɹ�;��Ҫ�����̼���APP����ת��APP
	}upgrader_type_t;
	typedef enum 
    {
		PACKET_CMD_INDEX     = PACK_DATA_INDEX	 ,//����ָ��λ��              	6
		PACKET_RESULT_INDEX           			 ,//������״̬�ظ�λ��			7
		PACKET_RDATA_INDEX            			 ,//�������ݻظ�������ʼλ��	8
		PACKET_FRAME_L_INDEX =PACKET_CMD_INDEX+1 ,//��������֡��λ				7
		PACKET_FRAME_H_INDEX					 ,//��������֡��λ				8
		PACKET_FIR_DATA_INDEX					 ,//��������λ��				9
    }en_packet_index_t;							  //������ָ�λ��
	typedef struct upGraderDate_t
	{	
		uint8_t  upCmd;							//����ָ��
		uint16_t frameNum;						//����֡
		uint16_t frameNumOld;					//������ʷ֡
		uint16_t fileCrc;						//�����̼�CRC
		uint32_t fileSize;						//�����̼���С
		uint32_t wFirSizeIndex;					//����д���Сλ��
		uint32_t wFirSize;						//����д���С
		uint32_t wFirAddr;						//����д���ַ
		uint8_t  wFirBuf[1024];
	}upGraderDate;

	typedef struct _stcSonarCfg_
	{
		uint8_t En1;							//1 ���巽ʽ 2 Modbus
		uint8_t En2;							//1 ���巽ʽ 2 Modbus
		uint8_t En3;							//1 ���巽ʽ 2 Modbus
		uint8_t En4;							//1 ���巽ʽ 2 Modbus
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
		//0x8F 0xFF:û�и��²�����BOOT��ת��APPP 0x9F:(App/Boot)����App�̼���ɣ�CRCУ��ɹ��������̼�������(Appʹ��������Bootʹ����ת)
		//0xAF:App����Boot�̼���ɣ�CRCУ��ɹ��������̼�������
		uint16_t upGoCmd;		
		uint16_t fileCrc;
		uint32_t fileSize;
		uint32_t backupsAddr;	//��������ַ
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
		uint8_t  len;							//����ָ��
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
	  uint32_t Port_BaudRate;   //������
	  uint8_t  Port_WordLength;	//����λ
	  uint8_t  Port_StopBits;	//ֹͣλ
	  uint8_t  Port_Parity;     //У��λ
	  uint8_t  Port_Mode;       //ģʽ	
	  uint8_t  Port_HFC ;		//����	  
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
		
		// 0�� USB�����������ϱ�    1��USB���������ϱ�    2��USB-CANת��    3��USB-RS485ת��    4��USB-RS232ת�� 	 5��USB-BLEת��     6��USB-UART4ת�� 	7��USB-UART1ת��    8:USB-RS485͸��ת��     9:USB-BLE͸��ת��
		// 10��BLE�����������ϱ�   11��BLE���������ϱ�   12��BLE-CANת��   13��BLE-RS485ת��   14��BLE-RS232ת�� 	15��NULL           16��BLE-UART4ת��   17��BLE-UART1ת��   18:BLE-RS485͸��ת��    19:BLE-RS232͸��ת��
		// 20��RS232�����������ϱ� 21��RS232���������ϱ� 22��RS232-CANת�� 23��RS232-RS485ת�� 24��NULL 			25��RS232-BLEת��  26��RS232-UART4ת�� 27��RS232-UART1ת�� 28:RS232-RS485͸��ת��  29:NULL
		// 30��UART1�����������ϱ� 31��UART1���������ϱ� 32��UART1-CANת�� 33��UART1-RS485ת�� 34��UART1-RS232ת��  35��UART1-BLEת��  36��UART1-UART4ת�� 37: NULL			   38:UART1-RS485͸��ת��  39:UART1-BLE͸��ת��
		uint8_t IsUsbLink;      
		uint8_t IsRosNodePub;		//0:ROSû������,1:ROS���ӳɹ�,2:IMU���ӻ������ϱ�ʹ��,3:ROS����ת��
		
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
#pragma pack(push)  //�������
#pragma pack(1) 	//�趨Ϊ1�ֽڶ���
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
		uint32_t tim_pos; 				//< Timestamp for position information λ����Ϣ��ʱ���*/
		uint16_t year;					//��
		uint8_t  month;					//��
		uint8_t  day;					//��
		uint8_t  hour;					//ʱ
		uint8_t  min;					//��
		uint8_t  sec;					//��
		int32_t lat;                 	/**< Latitude in 1E-7 degrees  γ��1E-7��*/
		int32_t lon;                 	/**< Longitude in 1E-7 degrees ����1E-7��*/
		int32_t alt;                 	/**< Altitude in 1E-3 meters (millimeters) above MSL  ���θ߶ȣ�����MSL 1E-3�ף����ף�*/
	} gps_report;
#pragma pack(pop)   //�ָ�����״̬
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

















