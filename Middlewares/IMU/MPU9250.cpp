#include "MPU9250.h"
MPU9250::MPU9250()
{

}
uint8_t MPU9250::init(IIC_TypeDef iic_)
{   
		uint8_t res=0;
		Wirempu9250.update(iic_);
		Wirempu9250.begin();
		Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
		delay_xms(100);
		Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
		Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
		Set_Accel_Fsr(3);					       	 	//加速度传感器,±2g
		Set_Rate(50);						       	 	//设置采样率50Hz
		Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
		Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
		Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
		Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
		res=Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
    if(res==MPU6500_ID1||res==MPU6500_ID2) //器件ID正确
    {
        Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
        Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
		Set_Rate(50);						       			//设置采样率为50Hz   
    }else return 1;
    res=Read_Byte(AK8963_ADDR,MAG_WIA);    			//读取AK8963 ID   
    if(res==AK8963_ID)
    {
        Write_Byte(AK8963_ADDR,MAG_CNTL2,0X01);		//复位AK8963
        Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//设置AK8963为单次测量
    }else return 1;
    return 0;
}
imu_check MPU9250::check()
{   
	uint8_t res=0;
	imu_check is_imu_check;
	is_imu_check.acc = 0;
	is_imu_check.gyro = 0;
	is_imu_check.magn = 0;
	res=Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
	if(res==MPU6500_ID1||res==MPU6500_ID2)
	{
		is_imu_check.acc = 1;
		is_imu_check.gyro = 1;
	}
	res=Read_Byte(AK8963_ADDR,MAG_WIA);
	if(res==AK8963_ID)
	{
		is_imu_check.magn = 1;
	}
	return	is_imu_check;
}
//设置MPU9250陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU9250::Set_Gyro_Fsr(uint8_t fsr)
{
	return Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,(fsr<<3)|3);//设置陀螺仪满量程范围  
}
//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU9250::Set_Accel_Fsr(uint8_t fsr)
{
	return Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU9250::Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU9250::Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}
//得到温度值
//返回值:温度值(扩大了100倍)
short MPU9250::Get_Temperature(void)
{
	uint8_t buf[2]; 
	short raw;
	float temp;
	Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
	raw=((uint16_t)buf[0]<<8)|buf[1];  
	temp=21+((float)raw)/333.87;  
	return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
imu_data_s MPU9250::Get_Gyroscope(void)
{   
	imu_data_s imu_data_g;
    uint8_t buf[6],res; 
	res=Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		imu_data_g.x=(((uint16_t)buf[0])<<8)|buf[1];  
		imu_data_g.y=(((uint16_t)buf[2])<<8)|buf[3];  
		imu_data_g.z=(((uint16_t)buf[4])<<8)|buf[5];
	} 	
    return imu_data_g;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
imu_data_s MPU9250::Get_Accelerometer(void)
{   
	imu_data_s imu_data_a;
    uint8_t buf[6],res;  
	res=Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		imu_data_a.x=(((uint16_t)buf[0])<<8)|buf[1];  
		imu_data_a.y=(((uint16_t)buf[2])<<8)|buf[3];  
		imu_data_a.z=(((uint16_t)buf[4])<<8)|buf[5];
	} 	
    return imu_data_a;
}
//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
imu_data_s MPU9250::Get_Magnetometer(void)
{   
	imu_data_s imu_data_m;
    uint8_t buf[6],res;  
 	res=Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		imu_data_m.x=(((uint16_t)buf[1])<<8)|buf[0];  
		imu_data_m.y=(((uint16_t)buf[3])<<8)|buf[2];  
		imu_data_m.z=(((uint16_t)buf[5])<<8)|buf[4];
	} 	 
	Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    return imu_data_m;
}
geometry_msgs::Vector3 MPU9250::readAccelerometer(void)
{
	geometry_msgs::Vector3 accel;
	imu_data_s accel_s;
	accel_s = Get_Accelerometer();
	accel.x = accel_s.x *(float)MPU9250_ACCEL_SCALE * G_TO_ACCEL;
	accel.y = accel_s.y *(float)MPU9250_ACCEL_SCALE * G_TO_ACCEL;
	accel.z = accel_s.z *(float)MPU9250_ACCEL_SCALE * G_TO_ACCEL;
	return accel;
}
geometry_msgs::Vector3 MPU9250::readGyroscope(void)
{
	geometry_msgs::Vector3 gyro;
	imu_data_s gyro_s;
	gyro_s = Get_Gyroscope();
	gyro.x = gyro_s.x *(float)MPU9250_GYRO_SCALE * DEG_TO_RAD;
	gyro.y = gyro_s.y *(float)MPU9250_GYRO_SCALE * DEG_TO_RAD;
	gyro.z = gyro_s.z *(float)MPU9250_GYRO_SCALE * DEG_TO_RAD;
	return gyro;
}
geometry_msgs::Vector3 MPU9250::readMagnetometer(void)
{
	geometry_msgs::Vector3 mag;
	imu_data_s mag_s;
	mag_s = Get_Magnetometer();
	mag.x = mag_s.x *(float)MPU9250_MAG_SCALE * UTESLA_TO_TESLA;
	mag.y = mag_s.y *(float)MPU9250_MAG_SCALE * UTESLA_TO_TESLA;
	mag.z = mag_s.z *(float)MPU9250_MAG_SCALE * UTESLA_TO_TESLA;
	return mag;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU9250::Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    Wirempu9250.Start();
    Wirempu9250.Send_Byte((addr<<1)|0); 	   //发送器件地址+写命令
    if(Wirempu9250.Wait_Ack_Reply())           //等待应答
    {
        Wirempu9250.Stop();
        return 1;
    }
    Wirempu9250.Send_Byte(reg);         	   //写寄存器地址
    Wirempu9250.Wait_Ack_Reply();              //等待应答
    for(i=0;i<len;i++)
    {
        Wirempu9250.Send_Byte(buf[i]);  	   //发送数据
        if(Wirempu9250.Wait_Ack_Reply())       //等待ACK
        {
            Wirempu9250.Stop();
            return 1;
        }
    }
    Wirempu9250.Stop();
    return 0;
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU9250::Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    Wirempu9250.Start();				   //IIC_Start();
    Wirempu9250.Send_Byte((addr<<1)|0);   // IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(Wirempu9250.Wait_Ack_Reply())      //等待应答
    {
        Wirempu9250.Stop();
        return 1;
    }
    Wirempu9250.Send_Byte(reg);         		//写寄存器地址
    Wirempu9250.Wait_Ack_Reply();				//IIC_Wait_Ack();//等待应答
	Wirempu9250.Start();						//IIC_Start();                
    Wirempu9250.Send_Byte((addr<<1)|1); 		//发送器件地址+读命令
    Wirempu9250.Wait_Ack_Reply();               //等待应答
    while(len)
    {
        if(len==1)
			*buf=Wirempu9250.Read_Byte_Ack(0);	//读数据,发送nACK 
		else 
			*buf=Wirempu9250.Read_Byte_Ack(1);  //读数据,发送ACK  
		len--;
		buf++;  
    }
    Wirempu9250.Stop();                 		//产生一个停止条件
    return 0;       
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU9250::Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
	Wirempu9250.Start();
    Wirempu9250.Send_Byte((addr<<1)|0); 	   //发送器件地址+写命令
    if(Wirempu9250.Wait_Ack_Reply())           //等待应答,如果返回1失败则停止
    {
        Wirempu9250.Stop();
        return 1;
    }
    Wirempu9250.Send_Byte(reg);         	   //写寄存器地址
    Wirempu9250.Wait_Ack_Reply();              //等待应答
    Wirempu9250.Send_Byte(data);       	 	   //发送数据
    if(Wirempu9250.Wait_Ack_Reply())           //等待ACK，如果返回1失败则停止
    {
        Wirempu9250.Stop();
        return 1;
    }
    Wirempu9250.Stop();
    return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t MPU9250::Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t res;
    Wirempu9250.Start();
    Wirempu9250.Send_Byte((addr<<1)|0); 		//发送器件地址+写命令
    Wirempu9250.Wait_Ack_Reply();               //等待应答
    Wirempu9250.Send_Byte(reg);         		//写寄存器地址
    Wirempu9250.Wait_Ack_Reply();             	//等待应答
	Wirempu9250.Start();               
    Wirempu9250.Send_Byte((addr<<1)|1); 		//发送器件地址+读命令
    Wirempu9250.Wait_Ack_Reply();               //等待应答
    res=Wirempu9250.Read_Byte_Ack(0);			//读数据,发送nACK  
    Wirempu9250.Stop();                         //产生一个停止条件
    return res;  
}