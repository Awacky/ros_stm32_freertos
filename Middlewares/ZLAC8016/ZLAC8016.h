#ifdef __cplusplus


#ifndef ZLAC8016_H
#define ZLAC8016_H

#include "hw_config.h"

class ZLAC8016
{
  public:
	ZLAC8016();
	void UpdateParam(u8 Dev_Id);
    void DeviceModeSet(void);
	void DeviceSpeedTimeSet(void);
	void DeviceEnabled(void);
	void DeviceSpeedSet(int MotorRpm);
	void DeviceDisability(void);
	void DeviceReadInfo(void);  
  private:
	u8 DevId;
};

#endif
#endif


