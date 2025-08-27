#ifndef PID_H
#define PID_H

#include "hw_config.h"

#define constrain(amt,low,high) \
	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class PID
{
  public:
	PID();
	void update(int min_val, int max_val, float kp, float ki, float kd);
	int compute(int setpoint, int measured_value);
	void updateConstants(float kp, float ki, float kd);

  private:
    int min_val_;
    int max_val_;
    float kp_;
    float ki_;
    float kd_;
    float integral_;
    float derivative_;
    float prev_error_;
};

#endif
