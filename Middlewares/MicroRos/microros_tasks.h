/*
********************************************************************************
*
* Created on: March 15, 2022
*     Author: Nipun Dhananjaya Weerakkodi -> nipun.dhananjaya@gmail.com
********************************************************************************
*/

#ifndef __MICROROS_TASKS_H__
#define __MICROROS_TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_config.h"

void ros2_task_create(void);
void setRos2ImuValue(const stcTwist *srcAcc,const stcTwist *srcAngVel,const stcTwist *srcMag);
void setRos2BatteryValue(float srcVol,float srcCur,float srcTemp);
void setRos2SonarValue(const SonarDate *srcSonar_p);
void setRos2VelocityValue(const stcMotorVel *srcVel_p);
#ifdef __cplusplus
}
#endif
#endif /* __MICROROS_TASKS_H__ */

