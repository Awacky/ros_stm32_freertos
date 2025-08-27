#include "MPU9250.h"
MPU9250::MPU9250()
{

}
uint8_t MPU9250::init(IIC_TypeDef iic_)
{   
		uint8_t res=0;
		Wirempu9250.update(iic_);
		Wirempu9250.begin();
		Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
		delay_xms(100);
		Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
		Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps
		Set_Accel_Fsr(3);					       	 	//���ٶȴ�����,��2g
		Set_Rate(50);						       	 	//���ò�����50Hz
		Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
		Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
		Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
		Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
		res=Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //��ȡMPU6500��ID
    if(res==MPU6500_ID1||res==MPU6500_ID2) //����ID��ȷ
    {
        Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
        Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
		Set_Rate(50);						       			//���ò�����Ϊ50Hz   
    }else return 1;
    res=Read_Byte(AK8963_ADDR,MAG_WIA);    			//��ȡAK8963 ID   
    if(res==AK8963_ID)
    {
        Write_Byte(AK8963_ADDR,MAG_CNTL2,0X01);		//��λAK8963
        Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//����AK8963Ϊ���β���
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
	res=Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //��ȡMPU6500��ID
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
//����MPU9250�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU9250::Set_Gyro_Fsr(uint8_t fsr)
{
	return Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,(fsr<<3)|3);//���������������̷�Χ  
}
//����MPU9250���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU9250::Set_Accel_Fsr(uint8_t fsr)
{
	return Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU9250�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU9250::Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU9250::Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}
//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
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
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
	Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
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
//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU9250::Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    Wirempu9250.Start();
    Wirempu9250.Send_Byte((addr<<1)|0); 	   //����������ַ+д����
    if(Wirempu9250.Wait_Ack_Reply())           //�ȴ�Ӧ��
    {
        Wirempu9250.Stop();
        return 1;
    }
    Wirempu9250.Send_Byte(reg);         	   //д�Ĵ�����ַ
    Wirempu9250.Wait_Ack_Reply();              //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        Wirempu9250.Send_Byte(buf[i]);  	   //��������
        if(Wirempu9250.Wait_Ack_Reply())       //�ȴ�ACK
        {
            Wirempu9250.Stop();
            return 1;
        }
    }
    Wirempu9250.Stop();
    return 0;
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU9250::Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    Wirempu9250.Start();				   //IIC_Start();
    Wirempu9250.Send_Byte((addr<<1)|0);   // IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(Wirempu9250.Wait_Ack_Reply())      //�ȴ�Ӧ��
    {
        Wirempu9250.Stop();
        return 1;
    }
    Wirempu9250.Send_Byte(reg);         		//д�Ĵ�����ַ
    Wirempu9250.Wait_Ack_Reply();				//IIC_Wait_Ack();//�ȴ�Ӧ��
	Wirempu9250.Start();						//IIC_Start();                
    Wirempu9250.Send_Byte((addr<<1)|1); 		//����������ַ+������
    Wirempu9250.Wait_Ack_Reply();               //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)
			*buf=Wirempu9250.Read_Byte_Ack(0);	//������,����nACK 
		else 
			*buf=Wirempu9250.Read_Byte_Ack(1);  //������,����ACK  
		len--;
		buf++;  
    }
    Wirempu9250.Stop();                 		//����һ��ֹͣ����
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU9250::Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
	Wirempu9250.Start();
    Wirempu9250.Send_Byte((addr<<1)|0); 	   //����������ַ+д����
    if(Wirempu9250.Wait_Ack_Reply())           //�ȴ�Ӧ��,�������1ʧ����ֹͣ
    {
        Wirempu9250.Stop();
        return 1;
    }
    Wirempu9250.Send_Byte(reg);         	   //д�Ĵ�����ַ
    Wirempu9250.Wait_Ack_Reply();              //�ȴ�Ӧ��
    Wirempu9250.Send_Byte(data);       	 	   //��������
    if(Wirempu9250.Wait_Ack_Reply())           //�ȴ�ACK���������1ʧ����ֹͣ
    {
        Wirempu9250.Stop();
        return 1;
    }
    Wirempu9250.Stop();
    return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
uint8_t MPU9250::Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t res;
    Wirempu9250.Start();
    Wirempu9250.Send_Byte((addr<<1)|0); 		//����������ַ+д����
    Wirempu9250.Wait_Ack_Reply();               //�ȴ�Ӧ��
    Wirempu9250.Send_Byte(reg);         		//д�Ĵ�����ַ
    Wirempu9250.Wait_Ack_Reply();             	//�ȴ�Ӧ��
	Wirempu9250.Start();               
    Wirempu9250.Send_Byte((addr<<1)|1); 		//����������ַ+������
    Wirempu9250.Wait_Ack_Reply();               //�ȴ�Ӧ��
    res=Wirempu9250.Read_Byte_Ack(0);			//������,����nACK  
    Wirempu9250.Stop();                         //����һ��ֹͣ����
    return res;  
}