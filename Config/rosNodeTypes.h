#ifndef __ROSNODETYPES_H_
#define __ROSNODETYPES_H_

#include <ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/Twist.h>
#include <starrobot_msgs/Imu.h>
#include <starrobot_msgs/Velocities.h>
#include <starrobot_msgs/analog.h>
#include <starrobot_msgs/Servo.h>
#include <starrobot_msgs/Sonar.h>
#include <starrobot_msgs/info_show.h>
#include <starrobot_msgs/PID.h>
#include <starrobot_msgs/Relaid.h>
#include <starrobot_msgs/Upgrader.h>
#include "datatypes.h"

void initParam(ros::NodeHandle *nh_t);
#ifndef Custom
void showParamSet(ros::NodeHandle *nh_t);
#endif
//void registeMoveVoidFun(float (*MoveMotorCPR_t)(void));
#endif /* __ROS_NODE_H_ */



