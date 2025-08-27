
#include "moveBase_Task.h"
#include "osQueueSemap.h"
#include "ErrorManage_Task.h"
#include "Kinematics.h"
#include "motor.h"
#include "PID.h"
#include "led.h"
#include "ros_node.h"	
#include "modem.h"
#include "servo.h" 
#include "sonar.h"
#include "timeout.h"
#include "ws2812Run_task.h"
#include "OledShow_Task.h"
#include "hw_adc.h"
#include "canMotor.h"

#define DELTA_CNT  10
static xQueueHandle  TwistQueue;
PID motor1_pid;
PID motor2_pid;
PID motor3_pid;
PID motor4_pid;
Servo   servo1;
Servo   servo2;
Servo   servo3;
Servo   servo4;
vMotorStr motor1_Str;
vMotorStr motor2_Str;
vMotorStr motor3_Str;
vMotorStr motor4_Str;
Kinematics kinematics;
Kinematics::base Robot_base;
Kinematics::velocities current_vel;
static uint8_t escRpm2Pwm = 0;
static float wheelCircumference=0.0;
static stcTwist str_tp;
//static stcMoveVel strMV_tp;
stcMoveVel strMV_tp;
stcMotorDebug mDeb_p;
stcMotorVel mVel_p;
static float curSteeringAngle = 0;
static uint8_t pubInfo_Num = 0;
static uint16_t encPIDCnt = 1;
static float calibLinVelTarget = 0;
static float calibAngVelTarget = 0;
static float calibVelTarget_last = 0;
static float calibVelTarget_calc;
static float calibElapsedTime = 0;
static float deltaCalib_t = 0;
static uint8_t calibStatus = 0; 	// 1,У׼��,2У׼���,3У׼��ʱ
static uint8_t wheelsNum = 0;		//С����������
static float baseTurningRadius = 0;	//С����ת�뾶
static float progress_calc;			//У׼���ȼ���
static uint8_t Control_Input = 1;	//��������:1->��λ��,ROSָ��;0->ң����ָ��
//static stcIndepend moveIndepStr;
stcIndepend moveIndepStr;			//λ��ģʽ���ݽ������
static uint8_t moveControl=0;		//����ģʽ //1��������ģʽ,0�˶�ѧ����ģʽ
volatile uint8_t m_RunStatus;		//����״̬
static uint8_t MotorType_t = 0;
_Moto_Str Motor_Rpm;	
//���ת��һȦ�������ļ���ֵ = ppr * ���ٱ� * 4
static float Time_counts_per_rev = 1560.f;//����ת��һȦ���������ļ���ֵ
volatile int delta_ticks_1=0;
volatile int delta_ticks_2=0;
volatile int delta_ticks_3=0;
volatile int delta_ticks_4=0;
volatile int64_t gCurrentEnc1=0;
volatile int64_t gCurrentEnc2=0;
volatile int64_t gCurrentEnc3=0;
volatile int64_t gCurrentEnc4=0;	
volatile int64_t lastCurrentEnc1=0;
volatile int64_t lastCurrentEnc2=0;
volatile int64_t lastCurrentEnc3=0;
volatile int64_t lastCurrentEnc4=0;
volatile int64_t lastIndepEnc1=0;
volatile int64_t lastIndepEnc2=0;
volatile int64_t lastIndepEnc3=0;
volatile int64_t lastIndepEnc4=0;
volatile int DirEnc1 = 0;
volatile int DirEnc2 = 0;
volatile int DirEnc3 = 0;
volatile int DirEnc4 = 0;

static void moveBase_Manage(void);
static void toRosVel_Publish(void);
static void moveIndependent_Manage(stcIndepend *srcValue);

void BaseClassInit(void)
{
	memset(&strMV_tp,0,sizeof(stcMoveVel));
	memset(&moveIndepStr,0,sizeof(stcIndepend));
	m_RunStatus = RUN_NONE;
	escRpm2Pwm = 500/configParam.Max_RPM;
	Time_counts_per_rev = configParam.CountsPer;
	MotorType_t = configParam.MotorType;
//	delta_t = (configParam.Pwm_arr*configParam.Pwm_psc)/168; //84*200 = 16800/168 = 100us = 0.1ms ��һ���ж�  100*0.1ms = 10ms 
	wheelCircumference = configParam.WheelDiameter * PI;
//	calc_delta_t = (60*1000000*wheelCircumference)/(delta_t*DELTA_CNT);		 //1min = 60s = 60000000 us  T = 60000000 / 10000 = 6000
//	calc_delta_t = (60*1000000)/(delta_t*DELTA_CNT);		 //1min = 60s = 60000000 us  T = 60000000 / 10000 = 6000
//	calc_delta_t = 6000;
	//����������õ������Ľ��ٶȣ��� = ��E/��T  RPM = (��*30)/PI
	//d2,d4,a1,a2,o3,o4,t2,t4,m4
	switch(configParam.RobotType){
		case ROBOT_D2:{
			Robot_base = Kinematics::d2;
			wheelsNum = 2;
			baseTurningRadius = 360.f/(wheelsNum*PI*configParam.LRInterval);
		}break;
		case ROBOT_D4:{
			Robot_base = Kinematics::d4;
			wheelsNum = 4;
			baseTurningRadius = 360.f/(wheelsNum*PI*(configParam.LRInterval+configParam.FBInterval));
		}break;
		case ROBOT_A1:{
			Robot_base = Kinematics::a1;
			wheelsNum = 2;
			baseTurningRadius = 360.f/(wheelsNum*PI*(configParam.LRInterval+configParam.FBInterval));
		}break;
		case ROBOT_A2:{
			Robot_base = Kinematics::a2;
			wheelsNum = 1;
			baseTurningRadius = 360.f/(wheelsNum*PI*(configParam.LRInterval+configParam.FBInterval));
		}break;
		case ROBOT_O3:{
			Robot_base = Kinematics::o3;
			wheelsNum = 3;
			baseTurningRadius = 360.f/(wheelsNum*PI*(configParam.LRInterval+configParam.FBInterval));
		}break;
		case ROBOT_O4:{
			Robot_base = Kinematics::o4;
			wheelsNum = 4;
			baseTurningRadius = 360.f/(wheelsNum*PI*(configParam.LRInterval+configParam.FBInterval));
		}break;
		case ROBOT_M4:{
			Robot_base = Kinematics::m4;
			wheelsNum = 4;
			baseTurningRadius = 360.f/(wheelsNum*PI*(configParam.LRInterval+configParam.FBInterval));
		}break;
	}
	kinematics.update(Robot_base,configParam.Max_RPM,configParam.WheelDiameter,
					  configParam.LRInterval, configParam.FBInterval, configParam.RobotRadius, configParam.MaxSteerAngle);
	switch(configParam.RobotType){
		case ROBOT_D2:{//d2 t2 
			motor1_Str.mDir = configParam.M1.Motor;motor1_Str.mDirveType = MotorType_t;
			motor1_Str.mName = MOTOR1;motor1_Str.mId = configParam.M1.Id;DirEnc1 = configParam.M1.Encoder;
			motor2_Str.mDir = configParam.M2.Motor;motor2_Str.mDirveType = MotorType_t;
			motor2_Str.mName = MOTOR2;motor2_Str.mId = configParam.M2.Id;DirEnc2 = configParam.M2.Encoder;
			switch(MotorType_t){
				case M_CAN_1_1:
				case M_CAN_1_2:{
					canMotorInit(&motor1_Str);
				}break;
				case M_ESC:{
					servo1.update(SERVO1);servo1.init();
					servo2.update(SERVO2);servo2.init();
				}break;
				case M_ESC_ENC:{
					servo1.update(SERVO1);servo1.init();
					servo2.update(SERVO2);servo2.init();
					EncoderInit(ENCODER1,MotorType_t);
					EncoderInit(ENCODER2,MotorType_t);
				}break;
				case M_CAN_ENC:{
					EncoderInit(ENCODER1,MotorType_t);
					EncoderInit(ENCODER2,MotorType_t);
				}
				default:{
					MotorInit(MOTOR1,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
					MotorInit(MOTOR2,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
					EncoderInit(ENCODER1,MotorType_t);
					EncoderInit(ENCODER2,MotorType_t);
				}break;
			}
		}break;
		case ROBOT_A1:{//a1 ת����+�������ٵ�� ��֧��ESC			
			motor1_Str.mDir = configParam.M1.Motor;motor1_Str.mDirveType = MotorType_t;motor1_Str.mName = MOTOR1;
			motor2_Str.mDir = configParam.M2.Motor;motor2_Str.mDirveType = MotorType_t;motor2_Str.mName = MOTOR2;
			MotorInit(MOTOR1,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
			MotorInit(MOTOR2,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
			DirEnc1 = configParam.M1.Encoder;
			DirEnc2 = configParam.M2.Encoder;
			EncoderInit(ENCODER1,MotorType_t);
			EncoderInit(ENCODER2,MotorType_t);
			servo1.update(SERVO1);servo1.init();
			servo1.updateConstants(configParam.TurnInitAngle);
			configParam.perEnabled.servo1 = 0;
		}break;			
		case ROBOT_A2:{//a2 ת����+һ��������� ֻ֧��ESC
			servo1.update(SERVO1);servo1.init();
			servo2.update(SERVO2);servo2.init();
			if(MotorType_t== M_ESC_ENC){
				DirEnc1 = configParam.M1.Encoder;
				EncoderInit(ENCODER1,MotorType_t);		
			}
			servo1.updateConstants(configParam.TurnInitAngle);
			configParam.perEnabled.servo1 = 0;
			configParam.perEnabled.servo2 = 0;
		}break;
		case ROBOT_O3:{//o3
			if(MotorType_t== M_ESC || MotorType_t== M_ESC_ENC){
				servo1.update(SERVO1);servo1.init();
				servo2.update(SERVO2);servo2.init();
				servo3.update(SERVO3);servo3.init();
			}else{
				motor1_Str.mDir = configParam.M1.Motor;motor1_Str.mDirveType = MotorType_t;motor1_Str.mName = MOTOR1;
				motor2_Str.mDir = configParam.M2.Motor;motor2_Str.mDirveType = MotorType_t;motor2_Str.mName = MOTOR2;
				motor3_Str.mDir = configParam.M3.Motor;motor3_Str.mDirveType = MotorType_t;motor3_Str.mName = MOTOR3;
				MotorInit(MOTOR1,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
				MotorInit(MOTOR2,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
				MotorInit(MOTOR3,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
			}
			if(MotorType_t!= M_ESC){
				DirEnc1 = configParam.M1.Encoder;
				DirEnc2 = configParam.M2.Encoder;
				DirEnc3 = configParam.M3.Encoder;
				EncoderInit(ENCODER1,MotorType_t);
				EncoderInit(ENCODER2,MotorType_t);
				EncoderInit(ENCODER3,MotorType_t);
			}
		}break;
		case ROBOT_D4: //d4 t4
		case ROBOT_O4: //o4
		case ROBOT_M4:{//m4
			motor1_Str.mDir = configParam.M1.Motor;motor1_Str.mDirveType = MotorType_t;
			motor1_Str.mName = MOTOR1;motor1_Str.mId = configParam.M1.Id;DirEnc1 = configParam.M1.Encoder;
			motor2_Str.mDir = configParam.M2.Motor;motor2_Str.mDirveType = MotorType_t;
			motor2_Str.mName = MOTOR2;motor2_Str.mId = configParam.M2.Id;DirEnc2 = configParam.M2.Encoder;
			motor3_Str.mDir = configParam.M3.Motor;motor3_Str.mDirveType = MotorType_t;
			motor3_Str.mName = MOTOR3;;motor3_Str.mId = configParam.M3.Id;DirEnc3 = configParam.M3.Encoder;
			motor4_Str.mDir = configParam.M4.Motor;motor4_Str.mDirveType = MotorType_t;
			motor4_Str.mName = MOTOR4;;motor4_Str.mId = configParam.M4.Id;DirEnc4 = configParam.M4.Encoder;
			switch(MotorType_t){
				case M_ESC:{
					servo1.update(SERVO1);servo1.init();
					servo2.update(SERVO2);servo2.init();
					servo3.update(SERVO3);servo3.init();
					servo4.update(SERVO4);servo4.init();
				}break;
				case M_ESC_ENC:{
					servo1.update(SERVO1);servo1.init();
					servo2.update(SERVO2);servo2.init();
					servo3.update(SERVO3);servo3.init();
					servo4.update(SERVO4);servo4.init();
					EncoderInit(ENCODER1,MotorType_t);
					EncoderInit(ENCODER2,MotorType_t);
					EncoderInit(ENCODER3,MotorType_t);
					EncoderInit(ENCODER4,MotorType_t);
				}break;
				case M_CAN_1_2:{
				}break;
				default:{
					MotorInit(MOTOR1,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
					MotorInit(MOTOR2,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
					MotorInit(MOTOR3,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
					MotorInit(MOTOR4,configParam.Pwm_arr,configParam.Pwm_psc,MotorType_t);
					EncoderInit(ENCODER1,MotorType_t);
					EncoderInit(ENCODER2,MotorType_t);
					EncoderInit(ENCODER3,MotorType_t);
					EncoderInit(ENCODER4,MotorType_t);
				}break;
			}
		}break;
	}
	if(MotorType_t!= M_ESC || MotorType_t!= M_ESC_ENC){
		if(configParam.perEnabled.servo3 && ((configParam.ControlMode&0x06) == 0)){
			servo3.update(SERVO3);servo3.init();
		}
		if(configParam.perEnabled.servo4 && ((configParam.ControlMode&0x06) == 0)){
			servo4.update(SERVO4);servo4.init();
		}
	}
	if(configParam.sonarCfg.En1 == 1 && configParam.sonarCfg.En2 == 1){
		Sonar_Init(Sonar1);
		Sonar_Init(Sonar2);
	}else if(configParam.sonarCfg.En1 == 1){
		Sonar_Init(Sonar1);
	}else if(configParam.sonarCfg.En2 == 1){
		Sonar_Init(Sonar2);
	}
	Hw_ADC_Init(1,MotorType_t);
	resetOverEncData();
}
void setPidParam(void)
{
	switch(configParam.RobotType){
		case 1:{//d2 t2 
				motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
				motor2_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M2.K_p, configParam.p_M2.K_i,configParam.p_M2.K_d);
		}break;
		case 3:{//a1 ת����+�������ٵ��
				motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
				motor2_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M2.K_p, configParam.p_M2.K_i,configParam.p_M2.K_d);
		}break;			
		case 4:{//a2 ת����+һ���������
			if(MotorType_t == M_ESC_ENC){		
				motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
							  configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
			}
		}break;
		case 5:{//o3
				motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
				motor2_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M2.K_p, configParam.p_M2.K_i,configParam.p_M2.K_d);
				motor3_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M3.K_p, configParam.p_M3.K_i,configParam.p_M3.K_d);
		}break;
		case 2: //d4 t4
		case 6: //o4
		case 7:{//m4
				motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
				motor2_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M2.K_p, configParam.p_M2.K_i,configParam.p_M2.K_d);
				motor3_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M3.K_p, configParam.p_M3.K_i,configParam.p_M3.K_d);
				motor4_pid.update(-configParam.Max_PWM, configParam.Max_PWM, 
								  configParam.p_M4.K_p, configParam.p_M4.K_i,configParam.p_M4.K_d);
		}break;
	}
}


static void toRosVel_Publish(void)
{	
	if(configParam.PubVel_Hz !=0){
		pubInfo_Num++;
		if(configParam.InfoEnabled == 1 || configParam.MotorDebug){		//���� InfoEnabled �����info����
			infolinkSendPacket(&mDeb_p);								//PID���������,������Ϣ����ʱ����	
			if(pubInfo_Num >=10){
				motorVellinkSendPacket(&mVel_p);						//PID���������,�ٶ�������ʱ����				
				pubInfo_Num = 0;
			}
		}else{
			motorVellinkSendPacket(&mVel_p);							//���������,�ٶ����ݲ���ʱ����
			if(pubInfo_Num >=10){
				motorVellinkSendPacket(&mVel_p);						//���������,������Ϣ��ʱ����					
				pubInfo_Num = 0;
			}
		}
	}
}
static void moveBase_Manage(void)
{	
	if(moveControl){
		moveIndependent_Manage(&moveIndepStr);
	} else {
		Kinematics::rpm req_rpm;
		req_rpm = kinematics.getRPM(strMV_tp.VelX, strMV_tp.VelY, strMV_tp.VelZ);
		switch(configParam.RobotType){
			case ROBOT_D2:{	//d2 t2
				mDeb_p.M1.Expectations = req_rpm.motor1 ;
				mDeb_p.M2.Expectations = req_rpm.motor2 ;
				mDeb_p.M1.Feedback = Motor_Rpm.Current_Rpm1 * DirEnc1;
				mDeb_p.M2.Feedback = Motor_Rpm.Current_Rpm2 * DirEnc2;
				if((1 == configParam.MotorDebug) || (M_CAN_1_2 == MotorType_t)){
					mDeb_p.M1.Pwm_Out  =  mDeb_p.M1.Expectations;
					mDeb_p.M2.Pwm_Out  =  mDeb_p.M2.Expectations;
				}else{
					if(MotorType_t == M_ESC){
						mDeb_p.M1.Pwm_Out  = mDeb_p.M1.Expectations*escRpm2Pwm;
						mDeb_p.M2.Pwm_Out  = mDeb_p.M2.Expectations*escRpm2Pwm;
					}else{
						mDeb_p.M1.Pwm_Out  =  motor1_pid.compute(mDeb_p.M1.Expectations,mDeb_p.M1.Feedback);
						mDeb_p.M2.Pwm_Out  =  motor2_pid.compute(mDeb_p.M2.Expectations,mDeb_p.M2.Feedback);
					}
				}
				motor1_Str.dutyVal = mDeb_p.M1.Pwm_Out;
				motor2_Str.dutyVal = mDeb_p.M2.Pwm_Out;
				switch(MotorType_t){
					case M_ESC:
					case M_ESC_ENC:{
						servo1.spin(configParam.EscMedianValue,mDeb_p.M1.Pwm_Out,configParam.M1.Motor);
						servo2.spin(configParam.EscMedianValue,mDeb_p.M2.Pwm_Out,configParam.M2.Motor);
					}break;
					case M_CAN_1_2:
					case M_CAN_1_1:{
//						sdoWriteMotorSpin(&motor1_Str,0);
//						sdoWriteMotorSpin(&motor2_Str,0);
					}break;
					case M_CAN_ENC:{
						sdoWriteMotorSpin(&motor1_Str,0);
						sdoWriteMotorSpin(&motor2_Str,0);
					}break;
					default:{
						MotorSpin(&motor1_Str);
						MotorSpin(&motor2_Str);		
					}break;
				}
				mVel_p.EncM1 = Motor_Rpm.MotorEncoder1*wheelCircumference*DirEnc1 / Time_counts_per_rev;
				mVel_p.EncM2 = Motor_Rpm.MotorEncoder2*wheelCircumference*DirEnc2 / Time_counts_per_rev;
				mVel_p.EncM3 = 0;
				mVel_p.EncM4 = 0;
				current_vel = kinematics.getVelocities(mDeb_p.M1.Feedback,mDeb_p.M2.Feedback,0,0);
			}break;
			case ROBOT_A1:{	//a1 ת����+�������ٵ��
				mDeb_p.M3.Expectations = req_rpm.motor3 * configParam.M3.Motor;
				servo1.pos(mDeb_p.M3.Expectations);//ת����
				mDeb_p.M1.Expectations = req_rpm.motor1 ;
				mDeb_p.M2.Expectations = req_rpm.motor2 ;
				mDeb_p.M1.Feedback = Motor_Rpm.Current_Rpm1 *DirEnc1;
				mDeb_p.M2.Feedback = Motor_Rpm.Current_Rpm2 *DirEnc2;
				if(1 == configParam.MotorDebug){
					mDeb_p.M1.Pwm_Out  =  mDeb_p.M1.Expectations;
					mDeb_p.M2.Pwm_Out  =  mDeb_p.M2.Expectations;
				}else{
					mDeb_p.M1.Pwm_Out  =  motor1_pid.compute(mDeb_p.M1.Expectations,mDeb_p.M1.Feedback);
					mDeb_p.M2.Pwm_Out  =  motor2_pid.compute(mDeb_p.M2.Expectations,mDeb_p.M2.Feedback);
				}
				motor1_Str.dutyVal = mDeb_p.M1.Pwm_Out;
				motor2_Str.dutyVal = mDeb_p.M2.Pwm_Out;
				MotorSpin(&motor1_Str);
				MotorSpin(&motor2_Str);						
				mVel_p.EncM1 = Motor_Rpm.MotorEncoder1*wheelCircumference*DirEnc1 / Time_counts_per_rev;
				mVel_p.EncM2 = Motor_Rpm.MotorEncoder2*wheelCircumference*DirEnc2 / Time_counts_per_rev;
				mVel_p.EncM3 = 0;
				mVel_p.EncM4 = 0;
				current_vel = kinematics.getVelocities(curSteeringAngle,mDeb_p.M1.Expectations,mDeb_p.M2.Expectations);
			}break;
			case ROBOT_A2:{	//a2 ת����+һ���������
				mDeb_p.M2.Expectations = kinematics.Steering(strMV_tp.VelZ);
				mDeb_p.M2.Expectations = mDeb_p.M2.Expectations * configParam.M2.Motor;
				servo1.pos(mDeb_p.M2.Expectations);//ת����
				mDeb_p.M1.Expectations = req_rpm.motor1 ;
				if(1 == configParam.MotorDebug){
					mDeb_p.M1.Pwm_Out  = mDeb_p.M1.Expectations;
					if(M_ESC_ENC == MotorType_t  || M_ESC == MotorType_t){
						servo2.spin(configParam.EscMedianValue,mDeb_p.M1.Pwm_Out,configParam.M1.Motor);//�������
					} else {
						motor1_Str.dutyVal = mDeb_p.M1.Pwm_Out;
						MotorSpin(&motor1_Str);
					}
				} else {
					if(M_ESC_ENC == MotorType_t){	//ʹ�ñ�����
						mDeb_p.M1.Feedback = Motor_Rpm.Current_Rpm1 *DirEnc1;
						mDeb_p.M1.Pwm_Out  =  motor1_pid.compute(mDeb_p.M1.Expectations,mDeb_p.M1.Feedback);	
						servo2.spin(configParam.EscMedianValue,mDeb_p.M1.Pwm_Out,configParam.M1.Motor);//�������
						mVel_p.EncM1 = Motor_Rpm.MotorEncoder1*wheelCircumference*DirEnc1 / Time_counts_per_rev;
						current_vel = kinematics.getVelocities(curSteeringAngle,mDeb_p.M1.Expectations);					
					} else { 								//��ʹ�ñ�����
						mDeb_p.M1.Pwm_Out  = mDeb_p.M1.Expectations*escRpm2Pwm * configParam.M1.Motor;
						mDeb_p.M1.Feedback = 0;
						servo2.spin(configParam.EscMedianValue,mDeb_p.M1.Pwm_Out,configParam.M1.Motor);//�������
					}
				}
			}break;
			case ROBOT_O3:{	//o3
				mDeb_p.M1.Expectations = req_rpm.motor1 ;
				mDeb_p.M2.Expectations = req_rpm.motor2 ;
				mDeb_p.M3.Expectations = req_rpm.motor3 ;
				mDeb_p.M1.Feedback = Motor_Rpm.Current_Rpm1 *DirEnc1;
				mDeb_p.M2.Feedback = Motor_Rpm.Current_Rpm2 *DirEnc2;
				mDeb_p.M3.Feedback = Motor_Rpm.Current_Rpm3 *DirEnc3;
				if(1 == configParam.MotorDebug){
					mDeb_p.M1.Pwm_Out  =  mDeb_p.M1.Expectations;
					mDeb_p.M2.Pwm_Out  =  mDeb_p.M2.Expectations;
					mDeb_p.M3.Pwm_Out  =  mDeb_p.M3.Expectations;				
				}else{
					if(MotorType_t == M_ESC){
						mDeb_p.M1.Pwm_Out  = mDeb_p.M1.Expectations*escRpm2Pwm;
						mDeb_p.M2.Pwm_Out  = mDeb_p.M2.Expectations*escRpm2Pwm;
						mDeb_p.M3.Pwm_Out  = mDeb_p.M3.Expectations*escRpm2Pwm;	
					}else{
						mDeb_p.M1.Pwm_Out  =  motor1_pid.compute(mDeb_p.M1.Expectations,mDeb_p.M1.Feedback);
						mDeb_p.M2.Pwm_Out  =  motor2_pid.compute(mDeb_p.M2.Expectations,mDeb_p.M2.Feedback);
						mDeb_p.M3.Pwm_Out  =  motor3_pid.compute(mDeb_p.M3.Expectations,mDeb_p.M3.Feedback);
					}
				}
				if(MotorType_t == M_ESC || MotorType_t == M_ESC_ENC){	
					servo1.spin(configParam.EscMedianValue,mDeb_p.M1.Pwm_Out,configParam.M1.Motor);
					servo2.spin(configParam.EscMedianValue,mDeb_p.M2.Pwm_Out,configParam.M2.Motor);
					servo3.spin(configParam.EscMedianValue,mDeb_p.M3.Pwm_Out,configParam.M3.Motor);
				}else{
					motor1_Str.dutyVal = mDeb_p.M1.Pwm_Out;
					motor2_Str.dutyVal = mDeb_p.M2.Pwm_Out;
					motor3_Str.dutyVal = mDeb_p.M3.Pwm_Out;
					MotorSpin(&motor1_Str);
					MotorSpin(&motor2_Str);	
					MotorSpin(&motor3_Str);					
				}
				mVel_p.EncM1 = Motor_Rpm.MotorEncoder1*wheelCircumference*DirEnc1 / Time_counts_per_rev;
				mVel_p.EncM2 = Motor_Rpm.MotorEncoder2*wheelCircumference*DirEnc2 / Time_counts_per_rev;
				mVel_p.EncM3 = Motor_Rpm.MotorEncoder3*wheelCircumference*DirEnc3 / Time_counts_per_rev;
				mVel_p.EncM4 = 0;
				current_vel = kinematics.getVelocities(mDeb_p.M1.Feedback,mDeb_p.M2.Feedback,mDeb_p.M3.Feedback,mDeb_p.M4.Feedback);
			}break;
			case ROBOT_D4:	//d4 t4
			case ROBOT_O4:	//o4
			case ROBOT_M4:{	//m4
				mDeb_p.M1.Expectations = req_rpm.motor1 ;
				mDeb_p.M2.Expectations = req_rpm.motor2 ;
				mDeb_p.M3.Expectations = req_rpm.motor3 ;
				mDeb_p.M4.Expectations = req_rpm.motor4 ;
				mDeb_p.M1.Feedback = Motor_Rpm.Current_Rpm1 *DirEnc1;
				mDeb_p.M2.Feedback = Motor_Rpm.Current_Rpm2 *DirEnc2;
				mDeb_p.M3.Feedback = Motor_Rpm.Current_Rpm3 *DirEnc3;
				mDeb_p.M4.Feedback = Motor_Rpm.Current_Rpm4 *DirEnc4;
				if((1 == configParam.MotorDebug) || (M_CAN_1_2 == MotorType_t)){
					mDeb_p.M1.Pwm_Out  =  mDeb_p.M1.Expectations;
					mDeb_p.M2.Pwm_Out  =  mDeb_p.M2.Expectations;
					mDeb_p.M3.Pwm_Out  =  mDeb_p.M3.Expectations;
					mDeb_p.M4.Pwm_Out  =  mDeb_p.M4.Expectations;
				}else{
					if(MotorType_t==M_ESC){
						mDeb_p.M1.Pwm_Out  = mDeb_p.M1.Expectations*escRpm2Pwm;
						mDeb_p.M2.Pwm_Out  = mDeb_p.M2.Expectations*escRpm2Pwm;
						mDeb_p.M3.Pwm_Out  = mDeb_p.M3.Expectations*escRpm2Pwm;
						mDeb_p.M4.Pwm_Out  = mDeb_p.M4.Expectations*escRpm2Pwm;
					}else{					
						mDeb_p.M1.Pwm_Out  =  motor1_pid.compute(mDeb_p.M1.Expectations,mDeb_p.M1.Feedback);
						mDeb_p.M2.Pwm_Out  =  motor2_pid.compute(mDeb_p.M2.Expectations,mDeb_p.M2.Feedback);
						mDeb_p.M3.Pwm_Out  =  motor3_pid.compute(mDeb_p.M3.Expectations,mDeb_p.M3.Feedback);
						mDeb_p.M4.Pwm_Out  =  motor4_pid.compute(mDeb_p.M4.Expectations,mDeb_p.M4.Feedback);
					}
				}
				motor1_Str.dutyVal = mDeb_p.M1.Pwm_Out;
				motor2_Str.dutyVal = mDeb_p.M2.Pwm_Out;
				motor3_Str.dutyVal = mDeb_p.M3.Pwm_Out;
				motor4_Str.dutyVal = mDeb_p.M4.Pwm_Out;
				switch(MotorType_t){
					case M_ESC:
					case M_ESC_ENC:{
						servo1.spin(configParam.EscMedianValue,mDeb_p.M1.Pwm_Out,configParam.M1.Motor);
						servo2.spin(configParam.EscMedianValue,mDeb_p.M2.Pwm_Out,configParam.M2.Motor);
						servo3.spin(configParam.EscMedianValue,mDeb_p.M3.Pwm_Out,configParam.M3.Motor);
						servo4.spin(configParam.EscMedianValue,mDeb_p.M4.Pwm_Out,configParam.M4.Motor);
					}break;
					case M_CAN_1_2:{
					}break;
					default:{
						MotorSpin(&motor1_Str);
						MotorSpin(&motor2_Str);	
						MotorSpin(&motor3_Str);		
						MotorSpin(&motor4_Str);	
					}break;						
				}
				mVel_p.EncM1 = Motor_Rpm.MotorEncoder1*wheelCircumference*DirEnc1 / Time_counts_per_rev;
				mVel_p.EncM2 = Motor_Rpm.MotorEncoder2*wheelCircumference*DirEnc2 / Time_counts_per_rev;
				mVel_p.EncM3 = Motor_Rpm.MotorEncoder3*wheelCircumference*DirEnc3 / Time_counts_per_rev;
				mVel_p.EncM4 = Motor_Rpm.MotorEncoder4*wheelCircumference*DirEnc4 / Time_counts_per_rev;
				current_vel = kinematics.getVelocities(mDeb_p.M1.Feedback,mDeb_p.M2.Feedback,mDeb_p.M3.Feedback,mDeb_p.M4.Feedback);
			}break;
		}
		mVel_p.Vel_x = current_vel.linear_x;
		mVel_p.Vel_y = current_vel.linear_y;
		mVel_p.Vel_z = current_vel.angular_z;
		switch(m_RunStatus){
			case RUN_LIN_VEL_CALIB:{
				calibVelTarget_calc =(mVel_p.EncM1 +mVel_p.EncM2 +mVel_p.EncM3 +mVel_p.EncM4)/wheelsNum;
				deltaCalib_t = calibVelTarget_calc-calibVelTarget_last+calibLinVelTarget;
				progress_calc = deltaCalib_t/calibLinVelTarget;
				calibStatus = 1;
				if(calibVelTarget_calc>=calibVelTarget_last){
					calibElapsedTime = (float)getElapsedTime();
					strMV_tp.VelX =0;
					strMV_tp.VelY =0;
					strMV_tp.VelZ =0;
					calibStatus = 2;
				}
			}break;
			case RUN_ANG_VEL_CALIB:{
				calibVelTarget_calc =(mVel_p.EncM2+mVel_p.EncM4-mVel_p.EncM1-mVel_p.EncM3 )*baseTurningRadius;
				deltaCalib_t = calibVelTarget_calc-calibVelTarget_last+calibAngVelTarget;
				progress_calc = deltaCalib_t/calibAngVelTarget;
				calibStatus = 1;
				if(calibVelTarget_calc>=calibVelTarget_last){
					calibElapsedTime = (float)getElapsedTime();					
					strMV_tp.VelX =0;
					strMV_tp.VelY =0;
					strMV_tp.VelZ =0;
					calibStatus = 2;
				}
			}break;
		}
	}
}
static void MoveBase_HTask(void *pvParameters)
{	
	uint16_t cNum_t = 0;uint16_t cNum_t2 = 0;
	uint16_t pubFreq_t = 3000;
    for( ;; ) 
	{	
		if(configParam.PubVel_Hz == 0){
			pubFreq_t = 3000;	
		}else{
			pubFreq_t = 1000/(configParam.PubVel_Hz|1);
		}
		if(pdTRUE == xQueueReceive(TwistQueue, &str_tp,pubFreq_t)){
			cNum_t  = 0;
			cNum_t2 = 0;
			if(Control_Input){
				if(0 == strMV_tp.isLock){
					strMV_tp.VelX = str_tp.linearVelX;
					strMV_tp.VelY = str_tp.linearVelY;
					strMV_tp.VelZ = str_tp.angularVelZ;
				} else if((strMV_tp.isLock == 1) && (str_tp.linearVelX<=0)){
					//ǰ������������ͣ,ֻ�ܿ��ƺ��� strMV_tp.isLock:0�����ͣ,1������ͣ,2ǰ������,3���˼�ͣ��,4���˼���
					strMV_tp.VelX = str_tp.linearVelX;
					strMV_tp.VelY = str_tp.linearVelY;
					strMV_tp.VelZ = str_tp.angularVelZ;
				}else if((strMV_tp.isLock == 3) && (str_tp.linearVelX>=0)){
					//������������ͣ,ֻ�ܿ���ǰ��  strMV_tp.isLock:0�����ͣ,1������ͣ,2ǰ������,3���˼�ͣ��,4���˼���
					strMV_tp.VelX = str_tp.linearVelX;
					strMV_tp.VelY = str_tp.linearVelY;
					strMV_tp.VelZ = str_tp.angularVelZ;
				}
			}
			if(RUN_NONE == m_RunStatus){
			   m_RunStatus = RUN_VEL;
			}
		}
		if(strMV_tp.isLock == 2){															//ǰ������
			if(strMV_tp.VelX > 0){
				strMV_tp.VelX = strMV_tp.VelX - 0.2;
				if(strMV_tp.VelX<0.2){
					strMV_tp.VelX = 0.2;
				}
			}
		}
		if(strMV_tp.isLock == 4){															//���˼���
			if(strMV_tp.VelX < 0){
				strMV_tp.VelX = strMV_tp.VelX + 0.2;
				if(strMV_tp.VelX > -0.2){
					strMV_tp.VelX = -0.2;
				}
			}
		}
		if((strMV_tp.isLock == 1) && (strMV_tp.VelX >= 0 || str_tp.linearVelX >= 0)){		//ǰ����ͣ�ٶ�����Ϊ0
				strMV_tp.VelX = 0;
				strMV_tp.VelY = 0;
				strMV_tp.VelZ = 0;
		}
		if((strMV_tp.isLock == 3) && (strMV_tp.VelX <= 0 || str_tp.linearVelX <= 0)){		//���˼�ͣ�ٶ�����Ϊ0
				strMV_tp.VelX = 0;
				strMV_tp.VelY = 0;
				strMV_tp.VelZ = 0;
		}		
		if(configParam.ControlMode&0x01){													//����ģʽ����,1������ʽ��0������ʽ
			cNum_t++;			
		}
		if(0==strMV_tp.VelX && 0 == strMV_tp.VelY && 0== strMV_tp.VelZ && moveControl==0){  //�ٶ�Ϊ0ʱ�ı�m_RunStatus״̬
			cNum_t2++;	 
		}
		if(cNum_t >= 50 || cNum_t2>=50){
			strMV_tp.VelX = 0;
			strMV_tp.VelY = 0;
			strMV_tp.VelZ = 0;
			cNum_t = 0;cNum_t2=0;
			switch(m_RunStatus){
				case RUN_ANG_VEL_CALIB:
				case RUN_LIN_VEL_CALIB:{
				}break;
				default:{
					m_RunStatus = RUN_NONE;
				}break;
			}						
		}
		if(strMV_tp.VelZ>0){			//��ת
			setRunMode_Fun(1);			//����ת��ƹ���ģʽ
		}else if(strMV_tp.VelZ<0){		//��ת
			setRunMode_Fun(2);			//����ת��ƹ���ģʽ
		}else{
			setRunMode_Fun(0);			//����ת��ƹ���ģʽ
		}
		if(configParam.IsRosNodePub){
			toRosVel_Publish();
		}
	}
}
void mVelSen_Mange(stcATBuff *srcStrBuff)
{	
	#ifndef Custom
	uint16_t length_vel;
	length_vel = PACK_DATA_INDEX;
	FloatMutualChar(&srcStrBuff->DataBuff[length_vel],mVel_p.Vel_x,1,0);
	length_vel +=4;
	FloatMutualChar(&srcStrBuff->DataBuff[length_vel],mVel_p.Vel_y,1,0);
	length_vel +=4;
	FloatMutualChar(&srcStrBuff->DataBuff[length_vel],mVel_p.Vel_z,1,0);
	length_vel +=4;
	FloatMutualChar(&srcStrBuff->DataBuff[length_vel],mVel_p.EncM1,1,0);
	length_vel +=4;
	FloatMutualChar(&srcStrBuff->DataBuff[length_vel],mVel_p.EncM2,1,0);
	length_vel +=4;
	FloatMutualChar(&srcStrBuff->DataBuff[length_vel],mVel_p.EncM3,1,0);
	length_vel +=4;
	FloatMutualChar(&srcStrBuff->DataBuff[length_vel],mVel_p.EncM4,1,0);
	length_vel = 28;
	srcStrBuff->DataBuff[PACK_LEN_L_INDEX] = length_vel;
	srcStrBuff->DataBuff[PACK_LEN_H_INDEX] = length_vel>>8;
	srcStrBuff->length_t = sendProcessing(srcStrBuff->DataBuff,CMD_READ_VEL);
	#endif
}	
void debugInfoSen_Mange(stcATBuff *srcStrBuff)
{	
	#ifndef Custom
	uint16_t length_vel;	
	memcpy(&srcStrBuff->DataBuff[PACK_DATA_INDEX],&mDeb_p,sizeof(stcMotorDebug));
	length_vel = sizeof(stcMotorDebug);
	srcStrBuff->DataBuff[PACK_LEN_L_INDEX] = length_vel;
	srcStrBuff->DataBuff[PACK_LEN_H_INDEX] = length_vel>>8;
	srcStrBuff->length_t = sendProcessing(srcStrBuff->DataBuff,CMD_READ_SHOW);
	#endif
}
static void msgsSend_HTask(void *pvParameters)
{
	uint16_t length_vel;
	uint8_t sendFreq_t = 0;
	static uint16_t msgSleep_t = 100;
	stcATBuff velStr_p;
	for(;;){
		switch(m_RunStatus){
			case RUN_VEL:{
				mVelSen_Mange(&velStr_p);
				#ifndef Custom
				communicationSend_Struct(&velStr_p,0);
				#endif
				sendFreq_t++;
				if(sendFreq_t>10){
					debugInfoSen_Mange(&velStr_p);
					#ifndef Custom
					communicationSend_Struct(&velStr_p,0);
					#endif
					sendFreq_t = 0;
				}
				oledUpdataMotorVal(&mDeb_p,&mVel_p);
				msgSleep_t =10;
			}break;
			case RUN_RPM:
			case RUN_POINT:
			case RUN_PID_CALIB:{
				debugInfoSen_Mange(&velStr_p);
				#ifndef Custom
				communicationSend_Struct(&velStr_p,0);
				#endif
				sendFreq_t++;
				if(sendFreq_t>10){
					mVelSen_Mange(&velStr_p);
					#ifndef Custom
					communicationSend_Struct(&velStr_p,0);
					#endif
					sendFreq_t = 0;
				}
				msgSleep_t =10;
			}break;	
			#ifndef Custom			
			case RUN_LIN_VEL_CALIB:{
				mVelSen_Mange(&velStr_p);

				communicationSend_Struct(&velStr_p,0);
				
				debugInfoSen_Mange(&velStr_p);
				communicationSend_Struct(&velStr_p,0);
				
				length_vel = PACK_DATA_INDEX;
				FloatMutualChar(&velStr_p.DataBuff[length_vel],progress_calc,1,0);
				length_vel +=4;
				FloatMutualChar(&velStr_p.DataBuff[length_vel],deltaCalib_t,1,0);
				length_vel +=4;
				FloatMutualChar(&velStr_p.DataBuff[length_vel],calibElapsedTime,1,0);
				length_vel +=4;
				if(timeout_has_timeout()){
					calibStatus = 3;
					timeout_disable();
				}
				velStr_p.DataBuff[length_vel] = calibStatus;// 1,У׼��,2У׼���,3У׼��ʱ
				length_vel = 13;
				velStr_p.DataBuff[RESULTS_TYPE_INDEX] = READ_TRPE;
				velStr_p.DataBuff[PACK_LEN_H_INDEX] = length_vel>>8;
				velStr_p.DataBuff[PACK_LEN_L_INDEX] = length_vel;
				velStr_p.length_t = sendProcessing(velStr_p.DataBuff,CMD_LIN_VEL_CALIB);
				communicationSend_Struct(&velStr_p,0);
				if(calibStatus != 1){
					m_RunStatus = RUN_NONE;
					strMV_tp.VelX =0;
					strMV_tp.VelY =0;
					strMV_tp.VelZ =0;
				}
				msgSleep_t = 50;
				}break;
			case RUN_ANG_VEL_CALIB:{
				mVelSen_Mange(&velStr_p);
				communicationSend_Struct(&velStr_p,0);
			
				debugInfoSen_Mange(&velStr_p);
				communicationSend_Struct(&velStr_p,0);
				length_vel = PACK_DATA_INDEX;
				FloatMutualChar(&velStr_p.DataBuff[length_vel],progress_calc,1,0);
				length_vel +=4;
				FloatMutualChar(&velStr_p.DataBuff[length_vel],deltaCalib_t,1,0);
				length_vel +=4;
				FloatMutualChar(&velStr_p.DataBuff[length_vel],calibElapsedTime,1,0);
				length_vel +=4;
				if(timeout_has_timeout()){
					calibStatus = 3;
					timeout_disable();
				}
				velStr_p.DataBuff[length_vel] = calibStatus;// 1,У׼��,2У׼���,3У׼��ʱ
				length_vel = 13;
				velStr_p.DataBuff[PACK_LEN_H_INDEX] = length_vel>>8;
				velStr_p.DataBuff[PACK_LEN_L_INDEX] = length_vel;
				velStr_p.length_t = sendProcessing(velStr_p.DataBuff,CMD_ANG_VEL_CALIB);
				communicationSend_Struct(&velStr_p,0);
				if(calibStatus != 1){
					m_RunStatus = RUN_NONE;
					strMV_tp.VelX =0;
					strMV_tp.VelY =0;
					strMV_tp.VelZ =0;
				}
				msgSleep_t = 50;
				}break;
			#endif
			default:
				msgSleep_t = 100;
				break;
		}
		if(M_CAN_1_2 == MotorType_t){
			sdoWriteMotorSpin(&motor1_Str,1);
			vTaskDelay(20);
			sdoWriteMotorSpin(&motor2_Str,1);
			vTaskDelay(20);
			sdoReadCanMotorRpm(&motor1_Str,1);
			vTaskDelay(20);
			sdoReadCanMotorRpm(&motor2_Str,1);
			vTaskDelay(20);
			sdoReadCanMotorEncoder(&motor1_Str,1);
			vTaskDelay(20);
			sdoReadCanMotorEncoder(&motor2_Str,1);
			vTaskDelay(20);
		}
		if(msgSleep_t ==0){
			msgSleep_t = 10;
		}
		vTaskDelay(msgSleep_t);
	}
}

static void moveIndependent_Manage(stcIndepend *srcValue){
	uint8_t m_EnableName = srcValue->m_EnableName;
	uint8_t kk=0;
	for(kk=0;kk<4;kk++){
		if(m_EnableName&(1<<kk)){
			switch(kk){
				case 0:{
					mDeb_p.M1.Expectations = srcValue->m1_Value.setValue ;
					mDeb_p.M1.Feedback = Motor_Rpm.Current_Rpm1 *DirEnc1;
					if(0==srcValue->m1_Value.loop){
						motor1_Str.dutyVal  =  mDeb_p.M1.Expectations;
					}else{
						motor1_Str.dutyVal  =  motor1_pid.compute(mDeb_p.M1.Expectations,mDeb_p.M1.Feedback);
					}
					if(srcValue->m1_Value.mode){// 0:�ٶ�ģʽ 1:λ��
						if(abs(Motor_Rpm.MotorEncoder1 - lastIndepEnc1) >= srcValue->m1_Value.setValue2){
							motor1_Str.dutyVal = 0;
							srcValue->m1_Value.setValue = 0;
						}
					}
					MotorSpin(&motor1_Str);
				}break;
				case 1:{
					mDeb_p.M2.Expectations = srcValue->m2_Value.setValue ;
					mDeb_p.M2.Feedback = Motor_Rpm.Current_Rpm2 *DirEnc2;
					if(0==srcValue->m2_Value.loop){
						motor2_Str.dutyVal  =  mDeb_p.M2.Expectations;
					}else{
						motor2_Str.dutyVal  =  motor2_pid.compute(mDeb_p.M2.Expectations,mDeb_p.M2.Feedback);
					}
					if(srcValue->m2_Value.mode){// 0:�ٶ�ģʽ 1:λ��
						if(abs(Motor_Rpm.MotorEncoder2 - lastIndepEnc2) >= srcValue->m2_Value.setValue2){
							motor2_Str.dutyVal = 0;
							srcValue->m2_Value.setValue = 0;
						}
					}
					MotorSpin(&motor2_Str);
				}break;
				case 2:{
					mDeb_p.M3.Expectations = srcValue->m3_Value.setValue ;
					mDeb_p.M3.Feedback = Motor_Rpm.Current_Rpm3 *DirEnc3;
					if(0==srcValue->m3_Value.loop){
						motor3_Str.dutyVal  =  mDeb_p.M3.Expectations;
					}else{
						motor3_Str.dutyVal  =  motor3_pid.compute(mDeb_p.M3.Expectations,mDeb_p.M3.Feedback);
					}
					if(srcValue->m3_Value.mode){// 0:�ٶ�ģʽ 1:λ��
						if(abs(Motor_Rpm.MotorEncoder3 - lastIndepEnc3) >= srcValue->m3_Value.setValue2){
							motor3_Str.dutyVal = 0;
							srcValue->m3_Value.setValue = 0;
						}
					}
					MotorSpin(&motor3_Str);
				}break;
				case 3:{
					mDeb_p.M4.Expectations = srcValue->m4_Value.setValue ;
					mDeb_p.M4.Feedback = Motor_Rpm.Current_Rpm4 *DirEnc4;
					if(0==srcValue->m4_Value.loop){
						motor4_Str.dutyVal  =  mDeb_p.M4.Expectations;
					}else{
						motor4_Str.dutyVal  =  motor4_pid.compute(mDeb_p.M4.Expectations,mDeb_p.M4.Feedback);
					}
					if(srcValue->m4_Value.mode){// 0:�ٶ�ģʽ 1:λ��
						if(abs(Motor_Rpm.MotorEncoder4 - lastIndepEnc4) >= srcValue->m4_Value.setValue2){
							motor4_Str.dutyVal = 0;
							srcValue->m4_Value.setValue = 0;
						}
					}
					MotorSpin(&motor4_Str);
				}break;
			}
		}
	}	
}
#ifdef __cplusplus
extern "C" {
#endif
	static void moveDelay(uint16_t delay_t);
	void MoveBase_TaskInit(){	
		BaseClassInit();
		setPidParam();
		setMotorDebugMaxRpm();
		TwistQueue = xQueueCreate(10, sizeof(stcTwist));
		#ifndef Custom
		registeMoveVoidFun(1,applyMotorDir);
		registeMoveVoidFun(2,setMotorDebugMaxRpm);
		registeMoveUint8Fun(1,setRunStatus);
		registeMoveUint8Fun(2,updatePidParam);
		registeMoveUint8Fun(3,setmoveControlValue);
		registeMoveCalTarget(1,set_calibLinVelTarget);
		registeMoveCalTarget(2,set_calibAngVelTarget);
		registeMoveSetModeValue(setmoveIndepStrValue);
		registeMoveServo(servoPerform);
		registeMoveLinkTwistSend(twistlinkSendPacket);
		registeMoveMrCprFun(GetTime_counts_per_rev);
		#endif
		registeCanMotorValueFun(&Motor_Rpm);
		registeCanDelayFun(moveDelay);
		xTaskCreate(MoveBase_HTask,(const char *)"MoveBase_HTask",256, NULL,MoveBase_Pri, NULL);
		xTaskCreate(msgsSend_HTask,(const char *)"msgsSend_HTask",256, NULL,msgsSend_Pri, NULL);
	}
	static void moveDelay(uint16_t delay_t){
		vTaskDelay(delay_t);
	}
	void setBaseClassInit(){
		BaseClassInit();
	}
	void setBaseClassPid(){
		setPidParam();
	}
	void setEStopStatus(uint8_t status_t){			//strMV_tp.isLock:0�����ͣ,1������ͣ,2ǰ������,3���˼�ͣ��,4���˼���
		switch(status_t){							//0�����ͣ
			case 0:{
				strMV_tp.isLock = 0;
			}break;
			case 1:{								//ǰ������������ͣ,���Կ��ƺ���
				if(m_RunStatus == RUN_VEL && strMV_tp.VelX>=0){				
					setError_Fun(ESTOPIC_Er);
					strMV_tp.isLock = 1;			//ǰ������������ͣ,���Կ��ƺ���
				}
			}break;
			case 2:{								//ǰ��������������
				if(m_RunStatus == RUN_VEL && strMV_tp.VelX>=0){				
					strMV_tp.isLock = 2;			//ǰ��������������
				}				
			}break;
			case 3:{								//������������ͣ,���Կ���ǰ��
				if(m_RunStatus == RUN_VEL && strMV_tp.VelX <= 0){
					setError_Fun(ESTOPIC_Er);
					strMV_tp.isLock = 3;			//������������ͣ,���Կ���ǰ��		
				}
			}break;
			case 4:{								//��������������
				if(m_RunStatus == RUN_VEL && strMV_tp.VelX<=0){		
					strMV_tp.isLock = 4;			//��������������	
				}
			}break;
		}
	}
	void setMoveVelData(const stcMoveVel *src_t){ 	//src_t->isLock:0�����ͣ,1������ͣ,2ǰ������,3���˼�ͣ��,4���˼���,5ң��������
		if(0 == src_t->isLock){						//����ң�������ƺ󣬿����ٶ���Ϣ���п���
			Control_Input = 1;
		}
		if(0 == strMV_tp.isLock && 5 == src_t->isLock){
			Control_Input = 0;
			//û�д�����ͣ����������ң�ؿ���
			strMV_tp.VelX = src_t->VelX;
			strMV_tp.VelY = src_t->VelY;
			strMV_tp.VelZ = src_t->VelZ;
		} else if((strMV_tp.isLock == 1) && (src_t->VelX<=0)){
			//ǰ������������ͣ,ֻ�ܿ��ƺ��� strMV_tp.isLock:0�����ͣ,1������ͣ,2ǰ������,3���˼�ͣ��,4���˼���
			strMV_tp.VelX = src_t->VelX;
			strMV_tp.VelY = src_t->VelY;
			strMV_tp.VelZ = src_t->VelZ;
		}else if((strMV_tp.isLock == 3) && (src_t->VelX>=0)){
			//������������ͣ,ֻ�ܿ���ǰ��  strMV_tp.isLock:0�����ͣ,1������ͣ,2ǰ������,3���˼�ͣ��,4���˼���
			strMV_tp.VelX = src_t->VelX;
			strMV_tp.VelY = src_t->VelY;
			strMV_tp.VelZ = src_t->VelZ;
		}
		m_RunStatus = RUN_VEL;
	}
	void setMotorDebugMaxRpm(void){
		if(configParam.MotorDebug){
			kinematics.updateMaxRpmValue(configParam.Max_PWM);
		}else{
			kinematics.updateMaxRpmValue(configParam.Max_RPM);
		}
	}
	void servoPerform(uint16_t s1Data,uint16_t s2Data,uint16_t s3Data,uint16_t s4Data){
		if(configParam.perEnabled.servo1){
			servo1.pos(s1Data);
		}
		if(configParam.perEnabled.servo2){
			servo2.pos(s2Data);
		}
		if(configParam.perEnabled.servo3){
			servo3.pos(s3Data);
		}
		if(configParam.perEnabled.servo4){
			servo4.pos(s4Data);
		}	
	}
	void setSwerveData(uint8_t s1Data){	
		configParam.TurnInitAngle = s1Data;
		servo1.updateConstants(s1Data);
	}
	bool twistlinkSendPacket(const stcTwist *p){
		return xQueueSend(TwistQueue, p, 1000);	
	}
	void SetTime_counts_per_rev(const float intput){
		Time_counts_per_rev = intput;
	}
	float GetTime_counts_per_rev(void){
		return Time_counts_per_rev;
	}
	void setRunStatus(uint8_t input){
		m_RunStatus = input;
	}
	uint8_t getRunStatus(void){
		return m_RunStatus;
	}
	uint8_t setmoveIndepStrValue(stcIndepend srcValue){
		if(moveControl){
			m_RunStatus = RUN_RPM;
			moveIndepStr = srcValue;
			if(srcValue.m1_Value.mode){	//λ��ģʽ
				m_RunStatus = RUN_POINT;
				moveIndepStr.m1_Value.setValue2 = ((srcValue.m1_Value.setValue2 * Time_counts_per_rev) / 360.0);
				lastIndepEnc1 = Motor_Rpm.MotorEncoder1;
			}
			if(srcValue.m2_Value.mode){	//λ��ģʽ
				moveIndepStr.m2_Value.setValue2 = ((srcValue.m2_Value.setValue2 * Time_counts_per_rev) / 360.0);
				lastIndepEnc2 = Motor_Rpm.MotorEncoder2;
			}
			if(srcValue.m3_Value.mode){	//λ��ģʽ
				moveIndepStr.m3_Value.setValue2 = ((srcValue.m3_Value.setValue2 * Time_counts_per_rev) / 360.0);
				lastIndepEnc3 = Motor_Rpm.MotorEncoder3;
			}
			if(srcValue.m4_Value.mode){	//λ��ģʽ
				moveIndepStr.m4_Value.setValue2 = ((srcValue.m4_Value.setValue2 * Time_counts_per_rev) / 360.0);
				lastIndepEnc4 = Motor_Rpm.MotorEncoder4;
			}
			return 1;
		} else {
			return 0;
		}
	}
	void setmoveControlValue(uint8_t status){//1��������ģʽ,0�˶�ѧ����ģʽ
		if(0==status){m_RunStatus = RUN_NONE;}
		moveControl = status;
	}
	uint8_t set_calibLinVelTarget(float input,float rVel){
		moveControl=0;
		if(m_RunStatus == RUN_NONE){
			calibLinVelTarget = input;
			strMV_tp.VelX = rVel;
			m_RunStatus = RUN_LIN_VEL_CALIB;
			calibStatus = 1;
			calibVelTarget_last =calibLinVelTarget + (mVel_p.EncM1+mVel_p.EncM2 +mVel_p.EncM3+mVel_p.EncM4)/wheelsNum;//����Ŀ��ֵ
			timeout_reset();
			timeout_configure(input/rVel*2000.f,0);
			return 1;
		}else{
			return 0;
		}
	}
	float get_calibLinVelTarget(float input){
		calibLinVelTarget = input;
	}
	void applyMotorDir(void){
		motor1_Str.mDir = configParam.M1.Motor;
		motor2_Str.mDir = configParam.M2.Motor;
		motor3_Str.mDir = configParam.M3.Motor;
		motor4_Str.mDir = configParam.M4.Motor;
		DirEnc1 = configParam.M1.Encoder;
		DirEnc2 = configParam.M2.Encoder;
		DirEnc3 = configParam.M3.Encoder;
		DirEnc4 = configParam.M4.Encoder;
	}
	uint8_t set_calibAngVelTarget(float input,float rVel){	
		moveControl=0;
		if(m_RunStatus == RUN_NONE){
			calibAngVelTarget = input;
			strMV_tp.VelZ = rVel;
			m_RunStatus = RUN_ANG_VEL_CALIB;
			calibStatus = 1;
			calibVelTarget_last =calibAngVelTarget + (mVel_p.EncM2+mVel_p.EncM4-mVel_p.EncM1-mVel_p.EncM3 )*baseTurningRadius;
			timeout_reset();
			timeout_configure(((input*PI)/(rVel*180))*2000.f,0);
			return 1;
		}else{
			return 0;
		}
	}
	float get_calibAngVelTarget(void){
		return calibAngVelTarget;
	}
	void resetOverEncData(void){
		memset(&Motor_Rpm,0,sizeof(_Moto_Str));
	}
	void updatePidParam(uint8_t motorName){
		switch(motorName){
			case 1:{
				motor1_pid.updateConstants(configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
			}break;
			case 2:{
				motor2_pid.updateConstants(configParam.p_M2.K_p, configParam.p_M2.K_i,configParam.p_M2.K_d);
			}break;
			case 3:{
				motor3_pid.updateConstants(configParam.p_M3.K_p, configParam.p_M3.K_i,configParam.p_M3.K_d);
			}break;
			case 4:{
				motor4_pid.updateConstants(configParam.p_M4.K_p, configParam.p_M4.K_i,configParam.p_M4.K_d);
			}break;
		}
	}
	#ifdef STM32F40_41xxx
		void TIM6_DAC_IRQHandler(void)
		{   
			if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET) 
			{
				ledpwm_update_pwm();
				encPIDCnt++;
				if(encPIDCnt >= DELTA_CNT){
					////�������������� 100 * x ms
					if(MotorType_t != M_CAN_1_2){					
						gCurrentEnc1 = TIM5 -> CNT;
						gCurrentEnc2 = TIM2 -> CNT;
						gCurrentEnc3 = TIM3 -> CNT;
						gCurrentEnc4 = TIM4 -> CNT;

						delta_ticks_1 = gCurrentEnc1 - lastCurrentEnc1;
						if(delta_ticks_1>2147483647 ){//�ݼ��ٽ� 
							delta_ticks_1 = gCurrentEnc1 - 0xffffffff - lastCurrentEnc1;
						}
						if(delta_ticks_1<-2147483647){//�����ٽ�
							delta_ticks_1 = gCurrentEnc1 + 0xffffffff - lastCurrentEnc1;
						}
						lastCurrentEnc1 = gCurrentEnc1;
						
						delta_ticks_2 = gCurrentEnc2 - lastCurrentEnc2;
						if(delta_ticks_2>2147483647){//�ݼ��ٽ�
							delta_ticks_2 = gCurrentEnc2 - 0xffffffff - lastCurrentEnc2;
						}
						if(delta_ticks_2<-2147483647){//�����ٽ�
							delta_ticks_2 = gCurrentEnc2 + 0xffffffff - lastCurrentEnc2;
						}
						lastCurrentEnc2 = gCurrentEnc2;
						
						delta_ticks_3 = gCurrentEnc3 - lastCurrentEnc3;
						if(delta_ticks_3>32767){//�ݼ��ٽ�
							delta_ticks_3 = gCurrentEnc3 - 0xffff - lastCurrentEnc3;
						}
						if(delta_ticks_3<-32767){//�����ٽ�
							delta_ticks_3 = gCurrentEnc3 + 0xffff - lastCurrentEnc3;
						}
						lastCurrentEnc3 = gCurrentEnc3;
						
						delta_ticks_4 = gCurrentEnc4 - lastCurrentEnc4;
						if(delta_ticks_4>32767){//�ݼ��ٽ�
							delta_ticks_4 = gCurrentEnc4 - 0xffff - lastCurrentEnc4;
						}
						if(delta_ticks_4<-32767){//�����ٽ�
							delta_ticks_4 = gCurrentEnc4 + 0xffff - lastCurrentEnc4;
						}
						lastCurrentEnc4 = gCurrentEnc4;
						
						Motor_Rpm.MotorEncoder1 += delta_ticks_1;
						Motor_Rpm.MotorEncoder2 += delta_ticks_2;
						Motor_Rpm.MotorEncoder3 += delta_ticks_3;
						Motor_Rpm.MotorEncoder4 += delta_ticks_4;
						Motor_Rpm.Current_Rpm1 = (delta_ticks_1*600/Time_counts_per_rev);
						Motor_Rpm.Current_Rpm2 = (delta_ticks_2*600/Time_counts_per_rev);
						Motor_Rpm.Current_Rpm3 = (delta_ticks_3*600/Time_counts_per_rev);
						Motor_Rpm.Current_Rpm4 = (delta_ticks_4*600/Time_counts_per_rev);
					}
					moveBase_Manage();
					encPIDCnt = 1;
				}
			}	
			TIM_ClearITPendingBit(TIM6,TIM_IT_Update);  				
		}
	#elif  AT32F40x
		void TMR6_GLOBAL_IRQHandler(void){
			if(tmr_flag_get(TMR6, TMR_OVF_FLAG) != RESET)
			{
				encPIDCnt++;
				ledpwm_update_pwm();
				if(encPIDCnt >= DELTA_CNT){
		
					gCurrentEnc1 = TMR5 -> cval;
					gCurrentEnc2 = TMR2 -> cval;
					gCurrentEnc3 = TMR3 -> cval;
					gCurrentEnc4 = TMR4 -> cval;

					delta_ticks_1 = gCurrentEnc1 - lastCurrentEnc1;
					if(delta_ticks_1>2147483647 ){//�ݼ��ٽ� 
						delta_ticks_1 = TMR5 -> cval - 0xffffffff - lastCurrentEnc1;
					}
					if(delta_ticks_1<-2147483647){//�����ٽ�
						delta_ticks_1 = TMR5 -> cval + 0xffffffff - lastCurrentEnc1;
					}
					lastCurrentEnc1 = gCurrentEnc1;
					
					delta_ticks_2 = gCurrentEnc2 - lastCurrentEnc2;
					if(delta_ticks_2>2147483647){//�ݼ��ٽ�
						delta_ticks_2 = TMR2 -> cval - 0xffffffff - lastCurrentEnc2;
					}
					if(delta_ticks_2<-2147483647){//�����ٽ�
						delta_ticks_2 = TMR2 -> cval + 0xffffffff - lastCurrentEnc2;
					}
					lastCurrentEnc2 = gCurrentEnc2;
					
					delta_ticks_3 = gCurrentEnc3 - lastCurrentEnc3;
					if(delta_ticks_3>32767){//�ݼ��ٽ�
						delta_ticks_3 = TMR3 -> cval - 0xffff - lastCurrentEnc3;
					}
					if(delta_ticks_3<-32767){//�����ٽ�
						delta_ticks_3 = TMR3 -> cval + 0xffff - lastCurrentEnc3;
					}
					lastCurrentEnc3 = gCurrentEnc3;
					
					delta_ticks_4 = gCurrentEnc4 - lastCurrentEnc4;
					if(delta_ticks_4>32767){//�ݼ��ٽ�
						delta_ticks_4 = TMR4 -> cval - 0xffff - lastCurrentEnc4;
					}
					if(delta_ticks_4<-32767){//�����ٽ�
						delta_ticks_4 = TMR4 -> cval + 0xffff - lastCurrentEnc4;
					}
					lastCurrentEnc4 = gCurrentEnc4;
					
					Motor_Rpm.MotorEncoder1 += delta_ticks_1;
					Motor_Rpm.MotorEncoder2 += delta_ticks_2;
					Motor_Rpm.MotorEncoder3 += delta_ticks_3;
					Motor_Rpm.MotorEncoder4 += delta_ticks_4;
					
					Motor_Rpm.Current_Rpm1 = (delta_ticks_1*600/Time_counts_per_rev);
					Motor_Rpm.Current_Rpm2 = (delta_ticks_2*600/Time_counts_per_rev);
					Motor_Rpm.Current_Rpm3 = (delta_ticks_3*600/Time_counts_per_rev);
					Motor_Rpm.Current_Rpm4 = (delta_ticks_4*600/Time_counts_per_rev);
					moveBase_Manage();
					encPIDCnt = 1;
				}
				tmr_flag_clear(TMR6, TMR_OVF_FLAG);
			}
		}
	#endif

#ifdef __cplusplus
}
#endif


