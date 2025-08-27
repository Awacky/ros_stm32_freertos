#include "OledShow_Task.h"
#include "devActivate.h"
#include "osQueueSemap.h"
#include "oled.h"
#include "device_storage.h"
#define OLED_REAT 50
static uint8_t oledPoll=0;
static uint8_t oledPollVal=0;
static stcMotorDebug oled_mDeb;
static stcMotorVel oled_mVel;
static float oledBatty;
static float odleMcuTemp;
static gps_report oledGps;
static uint8_t IsUpMotor=1;
static uint8_t IsUpGps=1;
Hardware_Struct gethStr;
static void DefaultDisplay(void){ 
	uint8_t showBuff[64];
	uint8_t getspLength = 0;
	sprintf((char *)showBuff,"Welcome starRobot");
	OLED_ShowString(10,0,showBuff, 1);
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"Time:%s",__DATE__);
	OLED_ShowString(0,1,showBuff, 1);
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"%s","IMUHardware:MPU6050");
	OLED_ShowString(0,2,showBuff, 1);
	switch(configParam.MotorType){
		case 1:
			getspLength = sprintf ((char *)showBuff, "motorDrive:%s","A4950");
			break;
		case 2:
			getspLength = sprintf ((char *)showBuff, "motorDrive:%s","BTN79xx");
			break;
		case 3:
			getspLength = sprintf ((char *)showBuff, "motorDrive:%s","ESC");
			break;
		case 4:
			getspLength = sprintf ((char *)showBuff, "motorDrive:%s","ESC_ENCODER");
			break;
		default:
			getspLength = sprintf ((char *)showBuff, "%s","No Set Motor drive");
			break;
	}
	OLED_ShowString(0,3,showBuff, 1);
	memset(showBuff,0,sizeof(showBuff));
	switch(configParam.RobotType){
		case 1:
			getspLength = sprintf ((char *)showBuff, "robotBase:%s","d2");
			break;
		case 2:
			getspLength = sprintf ((char *)showBuff, "robotBase:%s","d4");
			break;
		case 3:
			getspLength = sprintf ((char *)showBuff, "robotBase:%s","a1");
			break;
		case 4:
			getspLength = sprintf ((char *)showBuff, "robotBase:%s","a2");
			break;
		case 5:
			getspLength = sprintf ((char *)showBuff, "robotBase:%s","o3");
			break;
		case 6:
			getspLength = sprintf ((char *)showBuff, "robotBase:%s","o4");
			break;
		case 7:
			getspLength = sprintf ((char *)showBuff, "robotBase:%s","m4");
			break;
		default:
			getspLength = sprintf ((char *)showBuff, "%s","No Set Robot type");
			break;
	}
	OLED_ShowString(0,4,showBuff, 1);
	#ifndef Custom
	getDeviceHardwareStr(&gethStr);
	#endif
	memset(showBuff,0,sizeof(showBuff));
	getspLength = snprintf((char *)showBuff,21,"Hv:%s",(char *)gethStr.Hardware_Ver);
	OLED_ShowString(0,5,showBuff, 1);
	memset(showBuff,0,sizeof(showBuff));
	getspLength = sprintf((char *)showBuff,"Sv:%s",(char *)gethStr.Software_Ver);
	OLED_ShowString(0,6,showBuff, 1);
	memset(showBuff,0,sizeof(showBuff));
	getspLength = snprintf((char *)showBuff,21,"SN:%s",(char *)gethStr.Device_SN);
	OLED_ShowString(0,7,showBuff, 1);
}
static uint8_t MotorInfoDisplay(void){
	if(IsUpMotor)return 0;
	uint8_t showBuff[64];
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"Bat:%.1f Temp:%.1f",oledBatty,odleMcuTemp);
	OLED_ShowString(2,0,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"Vx:%.1f,y:%.1f,z:%.1f",oled_mVel.Vel_x,oled_mVel.Vel_y,oled_mVel.Vel_z);
	OLED_ShowString(0,1,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"M1  M2  M3  M4");
	OLED_ShowString(28,2,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"Pwm:%3d %3d %3d %3d",oled_mDeb.M1.Pwm_Out,oled_mDeb.M2.Pwm_Out,oled_mDeb.M3.Pwm_Out,oled_mDeb.M4.Pwm_Out);
	OLED_ShowString(0,3,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"Exp:%3d %3d %3d %3d",oled_mDeb.M1.Expectations,oled_mDeb.M2.Expectations,
										oled_mDeb.M3.Expectations,oled_mDeb.M4.Expectations);
	OLED_ShowString(0,4,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"Fee:%3d %3d %3d %3d",oled_mDeb.M1.Feedback,oled_mDeb.M2.Feedback,
										oled_mDeb.M3.Feedback,oled_mDeb.M4.Feedback);
	OLED_ShowString(0,5,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"eM1:%.2f  eM2:%.2f",oled_mVel.EncM1,oled_mVel.EncM2);
	OLED_ShowString(0,6,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"eM3:%.2f  eM4:%.2f",oled_mVel.EncM3,oled_mVel.EncM4);
	OLED_ShowString(0,7,showBuff, 1);
	return 1;
}
static uint8_t GpsInfoDisplay(void){
	if(IsUpGps) return 0;
	uint8_t showBuff[64];
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"TimerPos:%d",oledGps.tim_pos);
	OLED_ShowString(0,0,showBuff, 1);
	
	sprintf((char *)showBuff,"Date:%d-%d-%d %d:%d%d",oledGps.year,oledGps.month,oledGps.day,oledGps.hour,oledGps.min,oledGps.sec);
	OLED_ShowString(0,1,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"Longitude:%.3f",(float)(oledGps.lon*1e-7));
	OLED_ShowString(0,2,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"Latitude:%.3f",(float)(oledGps.lat*1e-7));
	OLED_ShowString(0,3,showBuff, 1);
	
	memset(showBuff,0,sizeof(showBuff));
	sprintf((char *)showBuff,"hMSL:%.3f",(float)(oledGps.alt*1e-3));
	OLED_ShowString(0,4,showBuff, 1);
	return 1;
}

static void OledShow_HTask(void *pvParameters){	
    for( ;; ) {	
		switch(oledPoll){
			case 0:{
				DefaultDisplay();
				oledPollVal++;
				if(oledPollVal>OLED_REAT){
					OLED_Clear();
					oledPollVal=0;
					oledPoll=1;
				}
			}break;
			case 1:{
				if(MotorInfoDisplay()){
					oledPollVal++;
				}else{
					oledPollVal=0;
					oledPoll=2;
				}
				if(oledPollVal>4*OLED_REAT){
					IsUpMotor = 1;
					OLED_Clear();
					oledPollVal=0;
					oledPoll=2;
				}
			}break;
			case 2:{
				if(GpsInfoDisplay()){
					oledPollVal++;
				}else{
					oledPollVal=0;
					oledPoll=0;
				}
				if(oledPollVal>2*OLED_REAT){
					IsUpGps = 1;
					OLED_Clear();
					oledPollVal=0;
					oledPoll=0;
				}
			}
		}
		vTaskDelay(100);
		OLED_Refreash();
	}
}
#ifdef __cplusplus
extern "C" {
#endif
	void OledShow_TaskInit(){
//		OLED_Init(1);	
//		OLED_Clear();
//		xTaskCreate(OledShow_HTask,(const char *)"OledShow_HTask",128, NULL, OledShow_Pri, NULL);
	}
	void oledUpdataMotorVal(stcMotorDebug *srcData1,stcMotorVel *srcData2){
		IsUpMotor = 0;
		memcpy((uint8_t *)&oled_mDeb,(uint8_t *)srcData1,sizeof(stcMotorDebug));
		memcpy((uint8_t *)&oled_mVel,(uint8_t *)srcData2,sizeof(stcMotorVel));
	}
    void oledUpdataGpsVal(gps_report *srcVal){
		IsUpGps = 0;
		memcpy((uint8_t *)&oledGps,(uint8_t *)srcVal,sizeof(gps_report));
	}
	void oledUpdataBat(float src1,float src2){
		IsUpMotor = 0;
		oledBatty = src1;
		odleMcuTemp = src2;
	}
#ifdef __cplusplus
}
#endif
