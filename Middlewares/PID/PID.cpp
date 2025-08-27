#include "PID.h"
PID::PID()
{
	
}

void PID::update(int min_val, int max_val, float kp, float ki, float kd)
{	
	min_val_ = min_val;
	max_val_ = max_val;
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}

int PID::compute(int setpoint, int measured_value)
{	

  double error = 0;
  double pid = 0;

  //setpoint is constrained between min and max to prevent pid from having too much error
  if(setpoint == 0){
	  integral_ = 0;
	  derivative_ = 0;
	  prev_error_ = 0;
	  return 0;
  }
  error = setpoint - measured_value;
  if(abs(error)<0.1)error=0;
  integral_ += error;
  derivative_ = error - prev_error_;
  if(setpoint == 0 && error == 0){
    integral_ = 0;
  }
  pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
  prev_error_ = error;

  return constrain(pid, min_val_, max_val_);
}
void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
