#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _MOVEBASE_TASK_H_
	#define _MOVEBASE_TASK_H_
	#include "hw_config.h"
	#include "osQueueSemap.h"
	void MoveBase_TaskInit();
	void setBaseClassInit();
	void setBaseClassPid();
	
	void setMotorDebugMaxRpm(void);
	void servoPerform(uint16_t s1Data,uint16_t s2Data,uint16_t s3Data,uint16_t s4Data);
	void setSwerveData(uint8_t s1Data);
	void SetTime_counts_per_rev(const float intput);
	float GetTime_counts_per_rev(void);
	void setRunStatus(uint8_t input);
	
	void applyMotorDir(void);
	uint8_t getRunStatus(void);
	uint8_t set_calibLinVelTarget(float input,float rVel);
	float get_calibLinVelTarget(float input);
	uint8_t set_calibAngVelTarget(float input,float rVel);
	float get_calibAngVelTarget(void);	
	void setmoveControlValue(uint8_t status);
	uint8_t setmoveIndepStrValue(stcIndepend srcValue);
	bool twistlinkSendPacket(const stcTwist *p);
	void resetOverEncData(void);
	void updatePidParam(uint8_t motorName);
	#endif
#ifdef __cplusplus
}
#endif

