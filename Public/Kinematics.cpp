#include "Kinematics.h"
#include "PID.h"
Kinematics::Kinematics()
{
}
void Kinematics::update(base robot_base,int max_rpm, float wheel_diameter, 
float wheels_x_distance, float wheels_y_distance, float robot_radius,float max_steering_angle)
{    
	base_platform        = robot_base;                                  //车子型号
	max_rpm_             = max_rpm;
	wheels_x_distance_   = wheels_x_distance; 							//车子左右轮间距
	wheels_y_distance_   = wheels_y_distance;                           //车子前后轮间距
	wheel_circumference_ = PI * wheel_diameter;                         //轮子周长
	total_wheels_        = getTotalWheels(robot_base);                  //
	max_steering_angle_  = max_steering_angle;
	if(robot_radius == 1.0){
		robot_radius_ = sqrtf((wheels_x_distance_ * wheels_x_distance_)+(wheels_y_distance_*wheels_y_distance_));
	}else{
		robot_radius_ = robot_radius;                                //车子旋转半径
	}
}
Kinematics::rpm Kinematics::calculateRPM(float linear_x)
{
	Kinematics::rpm rpm;
	rpm.motor1 = ((linear_x * 60)/wheel_circumference_);
	rpm.motor2 = 0;
	rpm.motor3 = 0;
	rpm.motor4 = 0;
	rpm.motor1 = constrain(rpm.motor1,-max_rpm_,max_rpm_);
	return rpm;
}
Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{
    float linear_vel_x_mins;
    float linear_vel_y_mins;
    float angular_vel_z_mins;
    float tangential_vel;
    float x_rpm;
    float y_rpm;
    float tan_rpm;
    float linear_vel_min_motor1;
    float linear_vel_min_motor2;
    float linear_vel_min_motor3;
    float linear_vel_min_motor4;
    // RPM = (V/r)*(30/pi) = V * (30/(r*pi)) = V * 60 / (2*pi*r)
    
        //convert m/s to m/min
        linear_vel_x_mins = linear_x * 60;
        linear_vel_y_mins = linear_y * 60;

        //convert rad/s to rad/min
        angular_vel_z_mins = angular_z * 60;
        tangential_vel = angular_vel_z_mins * ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2));
        x_rpm = linear_vel_x_mins / wheel_circumference_;
        y_rpm = linear_vel_y_mins / wheel_circumference_;
        tan_rpm = tangential_vel / wheel_circumference_;
        Kinematics::rpm rpm;
        //calculate for the target motor RPM and direction
        //front-left motor
        rpm.motor1 = x_rpm - y_rpm - tan_rpm;
        rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

        //front-right motor
        rpm.motor2 = x_rpm + y_rpm + tan_rpm;
        rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

        //rear-left motor
        rpm.motor3 = x_rpm + y_rpm - tan_rpm;
        rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

        //rear-right motor
        rpm.motor4 = x_rpm - y_rpm + tan_rpm;
        rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);
        return rpm;
}
Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    Kinematics::rpm rpm;
	rpm = calculateRPM(linear_x, 0.0 , angular_z);
    return rpm;
}
Kinematics::velocities Kinematics::getVelocities(float steering_angle, int rpm1, int rpm2)
{
		Kinematics::velocities vel;
		return vel;
}
Kinematics::velocities Kinematics::getVelocities(float steering_angle, int rpm1)
{
		Kinematics::velocities vel;
		return vel;
}
Kinematics::velocities Kinematics::getVelocities(int rpm1, int rpm2, int rpm3)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    average_rps_x = ((float)(-rpm1 + rpm2) * sqrt(3.0) / total_wheels_) / 60; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    average_rps_y = ((float)(-rpm1 - rpm2 + rpm3*2)/ total_wheels_)/60;
    vel.linear_y  = average_rps_y * wheel_circumference_;

    average_rps_a = ((float)(rpm1 + rpm2 + rpm3) / total_wheels_) / 60;
    vel.angular_z =  (average_rps_a  * wheel_circumference_) / robot_radius_; //  rad/s
     
    return vel; 
}

Kinematics::velocities Kinematics::getVelocities(int rpm1, int rpm2, int rpm3, int rpm4)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

        //convert average revolutions per minute to revolutions per second
        average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels_)/60.f; // RPM
        vel.linear_x = average_rps_x * wheel_circumference_; // m/s

        //convert average revolutions per minute in y axis to revolutions per second
        average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / total_wheels_)/60.f; // RPM
        if(base_platform == m4)
            vel.linear_y = average_rps_y * wheel_circumference_; // m/s
        else
            vel.linear_y = 0;

        //convert average revolutions per minute to revolutions per second
        average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_)/60.f;
        vel.angular_z =  (average_rps_a * wheel_circumference_) / ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2)); //  rad/s

        return vel;
}
int Kinematics::getTotalWheels(base robot_base)
{
    switch(robot_base){
        case d2:             return 2;
        case d4:             return 4;
        case a1:             return 1;
        case a2:             return 2;
        case o3:             return 3;
        case o4:             return 4;
        case m4:             return 4;
        default:             return 2;
    }
}
float Kinematics::Steering(float steering_angle)
{
    return 0;
}
//下发的角速度变为角度，返回值得度
//      x            in_min     in_max       out_min            out_max
//当前的角速度值  
float Kinematics::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void  Kinematics::updateMaxRpmValue(int setValue){
	max_rpm_ = setValue;
}
