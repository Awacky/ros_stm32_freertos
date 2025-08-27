#ifndef _MPU6500_H_
#define _MPU6500_H_

#include "SoftwareIIC.h"
#include <geometry_msgs/Vector3.h>
#include "MPU9250.h"
class MPU6500
{   
	public:
		MPU6500();
		uint8_t init(IIC_TypeDef iic_);
		imu_check check();
		short Get_Temperature(void);
		geometry_msgs::Vector3 readAccelerometer(void);
		geometry_msgs::Vector3 readGyroscope(void);
		geometry_msgs::Vector3 readMagnetometer(void);
	private:
		imu_data_s Get_Gyroscope(void);
		imu_data_s Get_Accelerometer(void);
		imu_data_s Get_Magnetometer(void);
		uint8_t Set_Gyro_Fsr(uint8_t fsr);
		uint8_t Set_Accel_Fsr(uint8_t fsr);
		uint8_t Set_LPF(uint16_t lpf);
		uint8_t Set_Rate(uint16_t rate);
		uint8_t Read_Byte(uint8_t addr,uint8_t reg);
		uint8_t Write_Byte(uint8_t addr,uint8_t reg,uint8_t data);
		uint8_t Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
		uint8_t Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
		SoftwareIIC Wirempu6500;
};
#endif





