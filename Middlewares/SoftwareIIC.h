#ifdef __cplusplus
#ifndef __STM32_I2C_H
#define __STM32_I2C_H

#include "hw_config.h"
#include "WireBase.h"
#include "delay.h"

#define I2C_WRITE 	0
#define I2C_READ  	1
#define IIC_DELAY 	5  //4
#define IIC_DELAY2	2   //4
#define BUFFER_LENGTH 64
class SoftwareIIC : public WireBase
{   
	public:
		SoftwareIIC();
		~SoftwareIIC();
		void update(IIC_TypeDef iic_);                                                           	//指定使用哪个设备
		void begin(void);                                                                         	//初始化
		void sda_out(void);																			//SDA输出模式配置
		void sda_in(void);																			//SDA输入模式配置
		int sda_read();                                                                            	//读取SDA数据
		void i2c_Delay(void);
		void Start(void);																			//开始启动信号函数
		void Stop(void); 																			//停止信号函数
		int Wait_Ack_Reply(void);																	//等待ACK应答函数
		void Generate_Ack_Reply(void);                                                              //产生ACK应答函数
		void NoGenerate_Ack_Reply(void);															//不产生ACK应答函数
		uint8_t Read_Ack(void);																		//读取ACK
		void Write_OneByte(uint8_t Dat);															//写入一个字节
		uint8_t Read_OneByte(void);																	//读取一个字节										
		void WriteCmd(uint8_t Cmd);																	//写入指令
		void WriteDat(uint8_t IIC_Data);															//读取数据
		int Write_Addr_Data(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);					//向指定地址写入数据（数组）
		int Read_Addr_Data(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);					//读取指定地址的数据（数组）
		uint8_t Read_Byte_Ack(uint8_t ack);															//读取Ack应答模式的一个字节函数
		void Send_Byte(uint8_t txd);
		uint8_t Read_DeviceAddr_OneByte(uint8_t Addr,uint8_t addr);
		uint8_t process(); 
		uint8_t shift_in() ;
		void shift_out(uint8_t val) ;
		bool get_ack() ;
		void send_ack() ;
		void send_nack() ;
	private:
		IIC_TypeDef iic; 
};
#endif
#endif




