#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "datatypes.h"

class Kinematics
{
  public:
		enum base { d2,d4,a1,a2,o3,o4,m4};
		base base_platform;
		struct rpm
		{
		  int motor1;
		  int motor2;
		  int motor3;
		  int motor4;
		};
		struct pwm 
		{
		  int motor1;
		  int motor2;
		  int motor3;
		  int motor4;
		};
		struct velocities 
		{
		  float linear_x;
		  float linear_y;
		  float angular_z;
		};
		Kinematics();
		void update(base robot_base,int max_pwm, float wheel_diameter, float wheels_x_distance, 
		float wheels_y_distance, float robot_radius,float max_steering_angle);
		velocities getVelocities(float steering_angle, int rpm1);
		velocities getVelocities(float steering_angle, int rpm1, int rpm2);
		velocities getVelocities(int rpm1, int rpm2, int rpm3);
		velocities getVelocities(int rpm1, int rpm2, int rpm3, int rpm4);
		rpm getRPM(float linear_x, float linear_y, float angular_z);
		float Steering(float steering_angle);
		float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
		int getTotalWheels(base robot_base);
		void  updateMaxRpmValue(int setValue);
  private:
		rpm calculateRPM(float linear_x, float linear_y, float angular_z);
		rpm calculateRPM(float linear_x);                            			 //阿克曼单电机驱动使用
		int max_rpm_;
		float wheels_x_distance_;
		float wheels_y_distance_;
		float robot_radius_;
		float wheel_circumference_;
		int total_wheels_;
		float max_steering_angle_;
};

#endif
