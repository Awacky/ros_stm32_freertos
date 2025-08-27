#ifdef __cplusplus
extern "C" {
#endif
	#ifndef __ROS_NODE_H_
	#define __ROS_NODE_H_
	
	#include "hw_config.h"
	void ros_task_create(void);
	bool infolinkSendPacket(stcMotorDebug *p);
	bool motorVellinkSendPacket(stcMotorVel *p);
	uint8_t rosRelaid_pubFun(void *Src_t);
	#endif /* __ROS_NODE_H_ */
#ifdef __cplusplus
}
#endif

