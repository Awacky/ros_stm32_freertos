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
		void update(IIC_TypeDef iic_);                                                           	//ָ��ʹ���ĸ��豸
		void begin(void);                                                                         	//��ʼ��
		void sda_out(void);																			//SDA���ģʽ����
		void sda_in(void);																			//SDA����ģʽ����
		int sda_read();                                                                            	//��ȡSDA����
		void i2c_Delay(void);
		void Start(void);																			//��ʼ�����źź���
		void Stop(void); 																			//ֹͣ�źź���
		int Wait_Ack_Reply(void);																	//�ȴ�ACKӦ����
		void Generate_Ack_Reply(void);                                                              //����ACKӦ����
		void NoGenerate_Ack_Reply(void);															//������ACKӦ����
		uint8_t Read_Ack(void);																		//��ȡACK
		void Write_OneByte(uint8_t Dat);															//д��һ���ֽ�
		uint8_t Read_OneByte(void);																	//��ȡһ���ֽ�										
		void WriteCmd(uint8_t Cmd);																	//д��ָ��
		void WriteDat(uint8_t IIC_Data);															//��ȡ����
		int Write_Addr_Data(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);					//��ָ����ַд�����ݣ����飩
		int Read_Addr_Data(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);					//��ȡָ����ַ�����ݣ����飩
		uint8_t Read_Byte_Ack(uint8_t ack);															//��ȡAckӦ��ģʽ��һ���ֽں���
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




