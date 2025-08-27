#ifdef __cplusplus
extern "C" {
#endif
	#ifndef _OLEDSHOW_TASK_H_
	#define _OLEDSHOW_TASK_H_
	#include "hw_config.h"
	void OledShow_TaskInit();
	void oledUpdataGpsVal(gps_report *srcVal);
	void oledUpdataMotorVal(stcMotorDebug *srcData1,stcMotorVel *srcData2);
	void oledUpdataBat(float src1,float src2);
	#endif //_LED_H_
#ifdef __cplusplus
}
#endif




