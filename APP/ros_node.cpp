#include "ros_node.h"
#include "osQueueSemap.h"
#include "rosNodeTypes.h"

#include "moveBase_Task.h"
#include "devActivate.h"
#include "ErrorManage_Task.h"

#include "hw_adc.h"
#include "sonar.h"
#include "device_storage.h"
#include "gy85.h"
#include "MPU9250.h"
#include "mpu6050.h"
#include "MPU6500.h"
#include "config_param.h"
#include "OledShow_Task.h"
#include "modem.h"
#if defined ( Microros )
	#include "microros_tasks.h"
#endif

static xQueueHandle  motorVelQueue;
static xQueueHandle  infoPubQueue;
static uint8_t IsParamShow = 0;
static uint8_t rosUpStatus=0;
static uint8_t subTimer = 10;
static uint8_t subTimerNum = 0;
upGraderDate rosUpStruct;
MPU6050 imu_mpu6050;
MPU9250 imu_mpu9250;
MPU6500 imu_mpu6500;
Gy85 imu_gy85;
SonarDate DateS_t;

ros::NodeHandle nh;
starrobot_msgs::Imu        raw_imu_msg;
starrobot_msgs::Velocities raw_vel_msg;
starrobot_msgs::analog     raw_analog_msg;
starrobot_msgs::info_show  raw_info_show_msg;
starrobot_msgs::Sonar      raw_sonar_msg;
starrobot_msgs::Relaid     raw_Relaid_msg;
starrobot_msgs::Upgrader   raw_Up_msg;

void pid_callback( const starrobot_msgs::PID& pid);
void command_callback(const geometry_msgs::Twist& cmd_msg);
void swerve_callback(const std_msgs::Int16& swerve_servo);
void servo_callback(const starrobot_msgs::Servo& servo_data);
void relaid_callback(const starrobot_msgs::Relaid& data_t);
void upgrader_callback(const starrobot_msgs::Upgrader& data_t);
void handleParam_callback(const std_msgs::Int8& data_t);
void test_fun(void);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<starrobot_msgs::PID>  pid_sub("pid", pid_callback);
ros::Subscriber<std_msgs::Int16> swerve_sub("initial_angle",swerve_callback); 
ros::Subscriber<starrobot_msgs::Servo>servo_sub("servo",servo_callback);
ros::Subscriber<starrobot_msgs::Relaid>relaid_sub("setRelaid",relaid_callback); 
ros::Subscriber<std_msgs::Int8>handleParam_sub("handleParam",handleParam_callback); 
ros::Subscriber<starrobot_msgs::Upgrader>upgrader_sub("UpPC2Dev",upgrader_callback); 

ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("raw_analog", &raw_analog_msg);
ros::Publisher raw_info_show_pub("info_show", &raw_info_show_msg);
ros::Publisher raw_sonar_pub("sonar", &raw_sonar_msg);
ros::Publisher raw_relaid_pub("getRelaid", &raw_Relaid_msg);
ros::Publisher raw_upgrader_pub("UpDev2PC", &raw_Up_msg);

void imuDriveInit(void){
	switch(configParam.IMUType){
		case 1:
			imu_gy85.init();
		break;
		case 2:
			imu_mpu9250.init(IMU_N);
		break;
		case 3:
			imu_mpu9250.init(IMU_W);
		break;
		case 4:
			imu_mpu6050.init(IMU_N);
		break;
		case 5:
			imu_mpu6050.init(IMU_W);
		break;
		case 6:
			imu_mpu6500.init(IMU_N);
		break;
		case 7:
			imu_mpu6500.init(IMU_W);
		break;
	}
}
void command_callback(const geometry_msgs::Twist& cmd_msg){
	stcTwist str_p;
	str_p.linearVelX = cmd_msg.linear.x;
	str_p.linearVelY = cmd_msg.linear.y;
	str_p.angularVelZ = cmd_msg.angular.z;
	if(false ==twistlinkSendPacket( &str_p))
		nh.loginfo("twistlinkSendPacket error");
}
void pid_callback(const starrobot_msgs::PID& pid){ 
	configParam.MotorDebug = 2;
	switch(pid.mIndex){
		case 1:{
			configParam.p_M1.K_p = pid.p;
			configParam.p_M1.K_i = pid.i;
			configParam.p_M1.K_d = pid.d;
		}break;
		case 2:{
			configParam.p_M2.K_p = pid.p;
			configParam.p_M2.K_i = pid.i;
			configParam.p_M2.K_d = pid.d;
		}break;
		case 3:{
			configParam.p_M3.K_p = pid.p;
			configParam.p_M3.K_i = pid.i;
			configParam.p_M3.K_d = pid.d;
		}break;
		case 4:{
			configParam.p_M4.K_p = pid.p;
			configParam.p_M4.K_i = pid.i;
			configParam.p_M4.K_d = pid.d;
		}break;	
	}
	updatePidParam(pid.mIndex);
}
void swerve_callback(const std_msgs::Int16& swerve_servo){   
	setSwerveData((uint8_t)swerve_servo.data);
}
void servo_callback(const starrobot_msgs::Servo& servo_data){	
	servoPerform(servo_data.servo1,servo_data.servo2,servo_data.servo3,servo_data.servo4);					
}
void relaid_callback(const starrobot_msgs::Relaid& data_t){   
	configParam.IsRosNodePub = 3;
	#ifndef Custom
	InterfaceRelaid(data_t.redata,data_t.reDLC,data_t.reId,data_t.reType);
	#endif
}
void handleParam_callback(const std_msgs::Int8& data_t){
	switch(data_t.data){
		case 1:{
			configParamGiveSemaphore();
			configParam.MotorDebug = 0;
			nh.loginfo("Parameter saving succeeded");
		}break;
		case 2:{ 
			IsParamShow = 1;
		}break;
		case 3:{
			configParam.InfoEnabled = 1;
			nh.loginfo("Enable Motor Info");
		}break;
		case 4:{
			configParam.InfoEnabled = 0;
			nh.loginfo("Disable Motor Info");
		}break;
		case 5:{
			configParam.MotorDebug = 1;
			setMotorDebugMaxRpm();
			nh.loginfo("Enable Motor Debug");
		}break;
		case 6:{
			configParam.MotorDebug = 0;
			setMotorDebugMaxRpm();
			nh.loginfo("Disable Motor Debug");
		}break;
		case 7:{
			resetOverEncData();
			nh.loginfo("Reset Raw_vel Odom");
		}break;
		case 8:{
			SoftReset_fun();
		}break;
		case 9:{
			rosUpStatus = 1;
			nh.loginfo("Firmware Upgrade Mode");
		}break;
		case 10:{
			rosUpStatus = 0;
			nh.loginfo("Firmware Upgrade Mode Return");
		}break;
		default:{
			nh.logwarn("handleParam Topic Help document");
			nh.loginfo("1:Parameter saving");
			nh.loginfo("2:Viewing Configuration Parameters");
			nh.loginfo("3:Enable Motor Info");
			nh.loginfo("4:Disable Motor Info");
			nh.loginfo("5:Enable Motor Debug");
			nh.loginfo("6:Disable Motor Debug");
			nh.loginfo("7:Reset Raw_vel Odom");
			nh.loginfo("8:Software Reset");	
			nh.loginfo("9:Firmware Upgrade Mode");
			nh.loginfo("10:Firmware Upgrade Mode Return");
		}break;
	}
}
void upgrader_callback(const starrobot_msgs::Upgrader& data_t){
	rosUpStruct.upCmd = data_t.upCmd;
	subTimerNum = 1;
	switch(data_t.upCmd){
		case UP_HANDSHAKE:{
			rosUpStatus = 1;
			subTimer = 10;
			watchdogInit(1500);
			rosUpStruct.fileCrc = data_t.upCrc;
			rosUpStruct.fileSize = data_t.upPayLen;
		}break;
		case UP_BOOT_FLASH_CRC:
		case UP_APP_FLASH_CRC:{	
		
		}break;
		default:{
			rosUpStruct.wFirSize = data_t.upPayLen;
			rosUpStruct.frameNum = data_t.upFrameNum;
			memcmp(rosUpStruct.wFirBuf,data_t.upData,data_t.upPayLen);
		}break;
	}
//	raw_Up_msg.upCrc = upgraderProcessing(&rosUpStruct,0,2);
	raw_Up_msg.upCmd = data_t.upCmd;
	raw_Up_msg.upFrameNum = data_t.upFrameNum;
	raw_upgrader_pub.publish(&raw_Up_msg);
	if(UP_APP_FLASH_CRC  == raw_Up_msg.upCmd ||\
	   UP_BOOT_FLASH_CRC == raw_Up_msg.upCmd){
		subTimer = 10;	
		subTimerNum=0;	
	}
}
void baseInfo_Publish(const stcMotorDebug *InfoStr_p){	
	if(rosUpStatus==0){
		raw_info_show_msg.Motor1_Out_Pwm = InfoStr_p->M1.Pwm_Out;
		raw_info_show_msg.Motor2_Out_Pwm = InfoStr_p->M2.Pwm_Out;
		raw_info_show_msg.Motor3_Out_Pwm = InfoStr_p->M3.Pwm_Out;
		raw_info_show_msg.Motor4_Out_Pwm = InfoStr_p->M4.Pwm_Out;
		raw_info_show_msg.Motor1_Expectations = InfoStr_p->M1.Expectations;
		raw_info_show_msg.Motor2_Expectations = InfoStr_p->M2.Expectations;
		raw_info_show_msg.Motor3_Expectations = InfoStr_p->M3.Expectations;
		raw_info_show_msg.Motor4_Expectations = InfoStr_p->M4.Expectations;
		raw_info_show_msg.Motor1_Feedback = InfoStr_p->M1.Feedback;
		raw_info_show_msg.Motor2_Feedback = InfoStr_p->M2.Feedback;
		raw_info_show_msg.Motor3_Feedback = InfoStr_p->M3.Feedback;
		raw_info_show_msg.Motor4_Feedback = InfoStr_p->M4.Feedback;
		raw_info_show_pub.publish(&raw_info_show_msg);
	}
}
void motorVel_Publish(const stcMotorVel *mVel_p){	
	if(rosUpStatus==0){
		raw_vel_msg.linear_x = mVel_p->Vel_x;
		raw_vel_msg.linear_y = mVel_p->Vel_y;
		raw_vel_msg.angular_z = mVel_p->Vel_z;
		raw_vel_msg.encoder_motor1 = mVel_p->EncM1;
		raw_vel_msg.encoder_motor2 = mVel_p->EncM2;
		raw_vel_msg.encoder_motor3 = mVel_p->EncM3;
		raw_vel_msg.encoder_motor4 = mVel_p->EncM4;
		raw_vel_pub.publish(&raw_vel_msg);
	}
}
void sonar_Publish(const SonarDate *dSonar_p){	
	if(rosUpStatus==0){
		raw_sonar_msg.sonar1 = dSonar_p->Sonar1;
		raw_sonar_msg.sonar2 = dSonar_p->Sonar2;
		raw_sonar_msg.sonar3 = dSonar_p->Sonar3;
		raw_sonar_msg.sonar4 = dSonar_p->Sonar4;
		raw_sonar_pub.publish(&raw_sonar_msg);
	}
}
void battery_Publish(void){	
	stcATBuff sBatBuffr_p;
	raw_analog_msg.voltage_Bus = getSamplingValue(voltage_Bus);
	raw_analog_msg.temp_Mcu = getSamplingValue(temp_Mcu);
	if((configParam.IsRosNodePub == 1 || configParam.IsRosNodePub == 3) && (rosUpStatus==0)){
		if(configParam.ROSType){
		#if defined ( Microros )
			setRos2BatteryValue(raw_analog_msg.voltage_Bus,0,raw_analog_msg.temp_Mcu);				
		#endif
		}else{
			raw_battery_pub.publish(&raw_analog_msg);
		}
	}
	#ifndef Custom
	sBatBuffr_p.length_t = PACK_DATA_INDEX;
	FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_analog_msg.voltage_Bus,1,0);
	sBatBuffr_p.length_t += 4;
	FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_analog_msg.temp_Mcu,1,0);
	sBatBuffr_p.length_t  = 8;
	sBatBuffr_p.DataBuff[PACK_LEN_L_INDEX] = sBatBuffr_p.length_t;
	sBatBuffr_p.DataBuff[PACK_LEN_H_INDEX] = sBatBuffr_p.length_t>>8;
	sBatBuffr_p.length_t = sendProcessing(sBatBuffr_p.DataBuff,CMD_READ_BAT);
	switch(configParam.IsUsbLink){
		case RELAID_USBautoUR2:
		case RELAID_USBautoUR3:
		case RELAID_UR5autoUR3:
		case RELAID_UR2autoUR3:
		case RELAID_UR2autoUR5:	
		case RELAID_UR1autoUR3:
		case RELAID_UR1autoUR2:{
		}break;	
		default:{
			communicationSend_Struct(&sBatBuffr_p,0);
			oledUpdataBat(raw_analog_msg.voltage_Bus,raw_analog_msg.temp_Mcu);
		}break;
	}
	#endif
	vTaskDelay(300);
}

void check_imu(void){    
	imu_check imu_check_data;
	switch(configParam.IMUType)
	{
		case 1:
			imu_check_data = imu_gy85.check();
		break;
		case 2:
		case 3:
			imu_check_data = imu_mpu9250.check();
		break;
		case 4:
		case 5:
			imu_check_data = imu_mpu6050.check();
		break;
		case 6:
		case 7:
			imu_check_data = imu_mpu6500.check();
		break;
	}
    if (!imu_check_data.acc){
		setError_Fun(IMU_CHECK_Er);
        nh.logerror("Accelerometer NOT FOUND!");
    }   
    if (!imu_check_data.gyro){
		setError_Fun(IMU_CHECK_Er);
        nh.logerror("Gyroscope NOT FOUND!");
    }   
    if (!imu_check_data.magn){
		setError_Fun(IMU_CHECK_Er);
        nh.logerror("Magnetometer NOT FOUND!");
    }
}
void PublishImu(){
	stcATBuff sBatBuffr_p;
	if((configParam.IsRosNodePub != 0) && (rosUpStatus==0)){
		switch(configParam.IMUType){   
			case 1:
				raw_imu_msg.linear_acceleration = imu_gy85.readAccelerometer();
				raw_imu_msg.angular_velocity = imu_gy85.readGyroscope();
				raw_imu_msg.magnetic_field = imu_gy85.readMagnetometer();
			break;
			case 2:
			case 3:
				raw_imu_msg.linear_acceleration = imu_mpu9250.readAccelerometer();
				raw_imu_msg.angular_velocity = imu_mpu9250.readGyroscope();
				raw_imu_msg.magnetic_field = imu_mpu9250.readMagnetometer();
			break;
			case 4:
			case 5:
				raw_imu_msg.linear_acceleration = imu_mpu6050.readAccelerometer();
				raw_imu_msg.angular_velocity = imu_mpu6050.readGyroscope();
				raw_imu_msg.magnetic_field = imu_mpu6050.readMagnetometer();
			break;
			case 6:
			case 7:
				raw_imu_msg.linear_acceleration = imu_mpu6500.readAccelerometer();
				raw_imu_msg.angular_velocity = imu_mpu6500.readGyroscope();
				raw_imu_msg.magnetic_field = imu_mpu6500.readMagnetometer();
			break;
			default:break;
		}
		switch(configParam.IsRosNodePub){
			case 1:{
				if(configParam.ROSType){
				#if defined ( Microros )
					stcTwist srcAcc_t,srcAVel_t,srcMag_t;
					srcAcc_t.linearVelX 	= raw_imu_msg.linear_acceleration.x;
					srcAcc_t.linearVelY 	= raw_imu_msg.linear_acceleration.y;
					srcAcc_t.angularVelZ 	= raw_imu_msg.linear_acceleration.z;
					srcAVel_t.linearVelX 	= raw_imu_msg.angular_velocity.x;
					srcAVel_t.linearVelY 	= raw_imu_msg.angular_velocity.y;
					srcAVel_t.angularVelZ 	= raw_imu_msg.angular_velocity.z;
					srcMag_t.linearVelX 	= raw_imu_msg.magnetic_field.x;
					srcMag_t.linearVelY 	= raw_imu_msg.magnetic_field.y;
					srcMag_t.angularVelZ 	= raw_imu_msg.magnetic_field.z;
					setRos2ImuValue(&srcAcc_t,&srcAVel_t,&srcMag_t);
				#endif
				}else{
					raw_imu_pub.publish(&raw_imu_msg);
				}
			}break;
			#ifndef Custom
			case 2:{
				sBatBuffr_p.length_t = PACK_DATA_INDEX;
				FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_imu_msg.linear_acceleration.x,1,0);
				sBatBuffr_p.length_t += 4;
				FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_imu_msg.linear_acceleration.y,1,0);
				sBatBuffr_p.length_t += 4;
				FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_imu_msg.linear_acceleration.z,1,0);
				sBatBuffr_p.length_t += 4;
				FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_imu_msg.angular_velocity.x,1,0);
				sBatBuffr_p.length_t += 4;
				FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_imu_msg.angular_velocity.y,1,0);
				sBatBuffr_p.length_t += 4;
				FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_imu_msg.angular_velocity.z,1,0);
				sBatBuffr_p.length_t += 4;
				FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_imu_msg.magnetic_field.x,1,0);
				sBatBuffr_p.length_t += 4;
				FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_imu_msg.magnetic_field.y,1,0);
				sBatBuffr_p.length_t += 4;
				FloatMutualChar(&sBatBuffr_p.DataBuff[sBatBuffr_p.length_t],raw_imu_msg.magnetic_field.z,1,0);
				sBatBuffr_p.length_t += 4;
				sBatBuffr_p.length_t  -= PACK_DATA_INDEX;
				sBatBuffr_p.DataBuff[PACK_LEN_L_INDEX] = sBatBuffr_p.length_t;
				sBatBuffr_p.DataBuff[PACK_LEN_H_INDEX] = sBatBuffr_p.length_t>>8;
				sBatBuffr_p.length_t = sendProcessing(sBatBuffr_p.DataBuff,CMD_READ_IMU);
				communicationSend_Struct(&sBatBuffr_p,0);
			}break;
			#endif
			default:break;	
		}
	}	
} 


static void ShowHardwareStr(void){
	char HarVerBuff[64];
	Hardware_Struct gethStr;
	#ifndef Custom
	getDeviceHardwareStr(&gethStr);
	#endif
	nh.loginfo("************************************");
	sprintf(HarVerBuff," Release Time:%s %s",__DATE__,__TIME__);
	nh.loginfo(HarVerBuff);
	nh.loginfo("         IMUHardware:MPU6050        ");
	memset(HarVerBuff,0,sizeof(HarVerBuff));
	sprintf(HarVerBuff,"    HardwareVer:%s",(char *)gethStr.Hardware_Ver);
	nh.loginfo(HarVerBuff);
	memset(HarVerBuff,0,sizeof(HarVerBuff));
	sprintf(HarVerBuff,"    SoftwareVer:%s",(char *)gethStr.Software_Ver);
	nh.loginfo(HarVerBuff);
	memset(HarVerBuff,0,sizeof(HarVerBuff));
	sprintf(HarVerBuff,"    DeviceSN:%s",(char *)gethStr.Device_SN);
	nh.loginfo(HarVerBuff);
	memset(HarVerBuff,0,sizeof(HarVerBuff));
	if(gethStr.ActStatus){
		sprintf(HarVerBuff,"    DeviceState:%s","Ok activated");
		nh.loginfo(HarVerBuff);
	}else{
		sprintf(HarVerBuff,"    DeviceState:%s","No activated");
		nh.logerror(HarVerBuff);
	}
	nh.loginfo("        starrobot Connected!        ");
}

static void ros_node_init(void *pvParameters){	
	static uint8_t hStr_t = 1;
	nh.initNode();
	nh.subscribe(cmd_sub);
	nh.subscribe(pid_sub);
	nh.subscribe(servo_sub);
	nh.subscribe(swerve_sub);
	nh.subscribe(relaid_sub);
	nh.subscribe(handleParam_sub);
	nh.subscribe(upgrader_sub);
	
	nh.advertise(raw_vel_pub);
	nh.advertise(raw_imu_pub);
	nh.advertise(raw_battery_pub);
	nh.advertise(raw_info_show_pub);
	nh.advertise(raw_sonar_pub);
	nh.advertise(raw_relaid_pub);
	nh.advertise(raw_upgrader_pub);
	for(;;){
		while (!nh.connected()){	
			nh.spinOnce();
			hStr_t = 1;
			vTaskDelay(300);
		}
		if(hStr_t){
			watchdogInit(600);
			#ifndef Custom
			initParam(&nh);
			#endif
			ShowHardwareStr();
			vTaskDelay(100);
			#ifndef Custom
			showParamSet(&nh);
			#endif
			setBaseClassInit();
			setBaseClassPid();
			imuDriveInit();
			configParam.IsRosNodePub = 1;
			setMotorDebugMaxRpm();
			hStr_t = 0;
			watchdogInit(WATCHDOG_RESET_MS);
		}
		if(IsParamShow){
			IsParamShow = 0;
			ShowHardwareStr();
			#ifndef Custom
		    showParamSet(&nh);
			#endif
		}
		if(subTimerNum){			//升级超时处理,防止上位机上级到一半，关闭节点问题
			subTimerNum++;
			if(subTimerNum>150){	//30s内没有收到升级数据测认为超时
				nh.loginfo("Firmware Upgrade Timeout");
				subTimer = 20;		//升级超时后恢复订阅任务为100hz
				subTimerNum = 0;
				rosUpStatus = 0;
				watchdogInit(WATCHDOG_RESET_MS);
			}
		}
		nh.spinOnce();
		vTaskDelay((subTimer|10));	//确保任务最大为100Hz
	}
}
static void BaseInfoPub_Task(void *pvParameters){	
	stcMotorDebug InfoStr_p;
	for(;;){
		if (xQueueReceive(infoPubQueue, &InfoStr_p, portMAX_DELAY) == pdTRUE){
				if(configParam.ROSType){
				#if defined ( Microros )
					
				#endif
				}else{
					baseInfo_Publish(&InfoStr_p);
				}
			}
	}
}
static void SonaroPub_Task(void *pvParameters){	
	SonarDate DateS_t2;
	stcATBuff Sonar_p;
	uint8_t abnormal = 0;
	uint16_t pubSonFreq_t = 3000;
	uint16_t length_sonar=0;
	for(;;){
		if(configParam.PubSonar_Hz == 0){
			pubSonFreq_t = 3000;	
		}else{
			pubSonFreq_t = 1000/configParam.PubSonar_Hz;
		}
		if(configParam.sonarCfg.En1||configParam.sonarCfg.En2){
			length_sonar++;
			if(length_sonar*50<pubSonFreq_t){		//脉冲模式的超声波读取先发送读取指令,控制使能脉冲发送
				if(configParam.sonarCfg.En1== 1 || configParam.sonarCfg.En2 == 1){	//超声波1和超声波2是脉冲模式的
					startCollectingSonar();
				}
			} else {								//超声波数据处理
				length_sonar = 0;
				if(configParam.sonarCfg.En1 == 2 || configParam.sonarCfg.En2 == 2 ||
				   configParam.sonarCfg.En3 == 2 || configParam.sonarCfg.En4 == 2) {	//超声波1,2,3,4是RS485模式的
					DateS_t = getUartSonarValue();
				}
				if(configParam.sonarCfg.En1== 1 || configParam.sonarCfg.En2 == 1){		//超声波1,2是脉冲模式的
					DateS_t2 = getSonarValue();
					DateS_t.Sonar1 = DateS_t2.Sonar1;
					DateS_t.Sonar2 = DateS_t2.Sonar2;
				}
				if(abnormal == 0){			//30S内不启用超声波避障
					if(DateS_t.Sonar1<configParam.sonarCfg.DeceDist1 && \
						DateS_t.Sonar1>configParam.sonarCfg.EstopDist1 && \
						configParam.sonarCfg.En1){			//前超声波感应到减速距离
						setEStopStatus(2);	//减速
					}else if( (DateS_t.Sonar1<configParam.sonarCfg.EstopDist1 && \
						0 != DateS_t.Sonar1 && configParam.sonarCfg.En1)){			//前超声波感应到急停距离
						setEStopStatus(1);	//急停				
					}
					if(DateS_t.Sonar2<configParam.sonarCfg.DeceDist2 && \
						DateS_t.Sonar2>configParam.sonarCfg.EstopDist2 && \
						configParam.sonarCfg.En2){			//后超声波感应到减速距离
						setEStopStatus(4);	//减速
					}else if(DateS_t.Sonar2<configParam.sonarCfg.EstopDist2 && \
						0 != DateS_t.Sonar2 && configParam.sonarCfg.En2){			//后超声波感应到急停距离
						setEStopStatus(3);	//急停				
					}
					if((configParam.sonarCfg.En1 && (configParam.sonarCfg.En2==0) && \
						DateS_t.Sonar1>configParam.sonarCfg.DeceDist1) || \
					   (configParam.sonarCfg.En2 && (configParam.sonarCfg.En1==0) && \
						DateS_t.Sonar2>configParam.sonarCfg.DeceDist2) || \
					   (configParam.sonarCfg.En1 && configParam.sonarCfg.En2 && \
						DateS_t.Sonar1>configParam.sonarCfg.DeceDist1 && \
						DateS_t.Sonar2>configParam.sonarCfg.DeceDist2)){
						//S1使能并且S2不使能,则当S1大于减速距离解除急停
						//S2使能并且S1不使能,则当S2大于减速距离解除急停
						//S1,S2使能  ,则当S1,S2大于减速距离解除急停
						setEStopStatus(0);	//解除急停
						resetError_Fun(ESTOPIC_Er);
					}
				}
				if((configParam.sonarCfg.En1 && configParam.sonarCfg.En2 && \
					DateS_t.Sonar1<configParam.sonarCfg.EstopDist1 && \
					DateS_t.Sonar2<configParam.sonarCfg.EstopDist2)){
					//超声波1,2使能,并且超声波1,2都小于急停距离,则30S内不启用超声波避障
					setEStopStatus(0);	//解除急停	
					resetError_Fun(ESTOPIC_Er);
					abnormal = 1;	
				}
				if(abnormal){					
					abnormal++;
					if(abnormal>150){
						//200ms * 150 = 30S内不启用超声波避障
						abnormal = 0;
					}
				}
				if((configParam.IsRosNodePub == 1 || configParam.IsRosNodePub == 3) && (0 != configParam.PubSonar_Hz)){
					//configParam.IsRosNodePub 0:ROS没有连接,1:ROS连接成功,2:IMU可视化数据上报使能,3:ROS数据转发
					if(configParam.ROSType){
					#if defined ( Microros )
//					void setRos2ImuValue(const stcTwist *srcAcc,const stcTwist *srcAngVel,const stcTwist *srcMag);
//					void setRos2BatteryValue(float srcVol,float srcCur,float srcTemp);				
//					void setRos2VelocityValue(const stcMotorVel *srcVel_p);
						setRos2SonarValue(&DateS_t);
					#endif
					}else{
						sonar_Publish(&DateS_t);
					}
				}
				#ifndef Custom
				length_sonar = PACK_DATA_INDEX;
				FloatMutualChar(&Sonar_p.DataBuff[length_sonar],DateS_t.Sonar1,1,0);
				length_sonar +=4;
				FloatMutualChar(&Sonar_p.DataBuff[length_sonar],DateS_t.Sonar2,1,0);
				length_sonar +=4;
				FloatMutualChar(&Sonar_p.DataBuff[length_sonar],DateS_t.Sonar3,1,0);
				length_sonar +=4;
				FloatMutualChar(&Sonar_p.DataBuff[length_sonar],DateS_t.Sonar4,1,0);
				length_sonar  =32;
				Sonar_p.DataBuff[PACK_LEN_H_INDEX] = length_sonar>>8;
				Sonar_p.DataBuff[PACK_LEN_L_INDEX] = length_sonar;
				Sonar_p.length_t=sendProcessing(Sonar_p.DataBuff,CMD_READ_SONAR);
				switch(configParam.IsUsbLink){
					case RELAID_USBautoUR2:
					case RELAID_USBautoUR3:
					case RELAID_UR5autoUR3:
					case RELAID_UR2autoUR3:
					case RELAID_UR2autoUR5:	
					case RELAID_UR1autoUR3:
					case RELAID_UR1autoUR2:{
					}break;	
					default:{
						communicationSend_Struct(&Sonar_p,0);
					}break;
				}
				#endif
				length_sonar = 0;
			}
			vTaskDelay(50);
		}else{	
			vTaskDelay(3000);
		}
	}
}

static void ImuPub_Task(void *pvParameters){	
	uint8_t is_first = 1;
	uint16_t pubImuFreq_t = 3000; 
	imuDriveInit();
	delay_xms(100);
	check_imu();
	for( ;; ) 
	{	
		if(configParam.PubImu_Hz == 0){
			pubImuFreq_t = 3000;	
		}else{
			pubImuFreq_t = 1000/(configParam.PubImu_Hz|10);
		}
		if(is_first){
			check_imu();
			is_first = 0;
		}else{
			PublishImu();
		}
		vTaskDelay(pubImuFreq_t);
//		vTaskDelay(1000);
	}
}
static void VelPub_Task(void *pvParameters){
	stcMotorVel mVel_t;
	for(;;){
		if (xQueueReceive(motorVelQueue, &mVel_t, portMAX_DELAY) == pdTRUE){
			if(configParam.ROSType){
			#if defined ( Microros )			
				setRos2VelocityValue(&mVel_t);
			#endif
			}else{
				motorVel_Publish(&mVel_t);
			}
		}
	}
}
static void BatPub_Task(void *pvParameters){
	for(;;){
		battery_Publish();
		vTaskDelay(1000/(configParam.PubBat_Hz|10));
	}
}
#ifdef __cplusplus
extern "C" {
#endif
	void ros_task_create(void)
	{	
		#ifndef Custom
		ros_communication_bsp_init();
		#endif
		infoPubQueue = xQueueCreate(10, sizeof(stcMotorDebug));
		motorVelQueue = xQueueCreate(10, sizeof(stcMotorVel));
		#ifndef Custom
		registerComSendCallback(rosRelaid_pubFun,RELAID_ROS);
		#endif
		xTaskCreate(BaseInfoPub_Task,(const char *)"BaseInfoPub_Task",	256, NULL, BaseInfoPub_Pri, NULL);
		xTaskCreate(SonaroPub_Task,  (const char *)"SonaroPub_Task", 	256, NULL, SonaroPub_Pri, NULL);
		xTaskCreate(ImuPub_Task,	 (const char *)"ImuPub_Task",		256, NULL, ImuPub_Pri, NULL);
		xTaskCreate(VelPub_Task,	 (const char *)"VelPub_Task",		256, NULL, VelPub_Pri, NULL);
		xTaskCreate(BatPub_Task,	 (const char *)"BatPub_Task",		256, NULL, BatPub_Pri, NULL);
		if(configParam.ROSType){
			return; 
		}
    	xTaskCreate(ros_node_init,   (const char *)"rosNodeTask",    	256, NULL, RosSub_Pri, NULL);
	}
	bool infolinkSendPacket(stcMotorDebug *p)
	{
		if(infoPubQueue != NULL){
			return xQueueSend(infoPubQueue, p, 1000);	
		} else {
			return 0;
		}
	}
	bool motorVellinkSendPacket(stcMotorVel *p)
	{
		if(motorVelQueue != NULL){
			return xQueueSend(motorVelQueue, p, 1000);	
		} else {
			return 0;
		}
	}
	uint8_t rosRelaid_pubFun(void *Src_t)
	{
		rosRelaidrMsg *pMsg = (rosRelaidrMsg *)Src_t;
		if(configParam.IsRosNodePub == 3){
			raw_Relaid_msg.reDLC = pMsg->Value.length_t;
			raw_Relaid_msg.reId  = pMsg->DevId;
			raw_Relaid_msg.redata_length =  pMsg->Value.length_t;
			raw_Relaid_msg.redata =  pMsg->Value.DataBuff;
			raw_relaid_pub.publish(&raw_Relaid_msg);
			return 1;
		} 
		return 0;
	}
#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/************************** Coopyright (C) Jungleeee 2018 *******END OF FILE********/

