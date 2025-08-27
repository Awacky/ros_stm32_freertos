#include "SoftwareIIC.h"

Hw_GPIO_TypeDef* IIC_SCL_PORT[IICn] = {STARBOT_IMU_N_SCL_PORT,STARBOT_IMU_W_SCL_PORT, STARBOT_OLED_SCL_PORT};
Hw_GPIO_TypeDef* IIC_SDA_PORT[IICn] = {STARBOT_IMU_N_SDA_PORT, STARBOT_IMU_W_SDA_PORT, STARBOT_OLED_SDA_PORT};

const Hw_CLK_TypeDef  IIC_SCL_PORT_CLK[IICn] = {STARBOT_IMU_N_SCL_CLK, STARBOT_IMU_W_SCL_CLK, STARBOT_OLED_SCL_CLK};
const Hw_CLK_TypeDef  IIC_SDA_PORT_CLK[IICn] = {STARBOT_IMU_N_SDA_CLK, STARBOT_IMU_W_SDA_CLK, STARBOT_OLED_SDA_CLK};

const uint16_t  IIC_SCL_PIN[IICn] = {STARBOT_IMU_N_SCL_PIN, STARBOT_IMU_W_SCL_PIN, STARBOT_OLED_SCL_PIN};
const uint16_t  IIC_SDA_PIN[IICn] = {STARBOT_IMU_N_SDA_PIN, STARBOT_IMU_W_SDA_PIN, STARBOT_OLED_SDA_PIN};

SoftwareIIC::SoftwareIIC()
{

}
void SoftwareIIC::update(IIC_TypeDef iic_)
{
     iic = iic_;
}
void SoftwareIIC::begin(void)
{	
	tx_buf_idx = 0;
    tx_buf_overflow = false;
    rx_buf_idx = 0;  
    rx_buf_len = 0;
}
void SoftwareIIC::sda_out(void)
{
	SoftwareIIC_SDA_Out();
}
void  SoftwareIIC::sda_in(void)
{
	SoftwareIIC_SDA_In();
}
int SoftwareIIC::sda_read()
{
	return SoftwareIIC_SDA_Read();
}
void  SoftwareIIC::Start(void)
{	
	sda_out();
	SoftwareIIC_SDA_H();					//SDA_H
	SoftwareIIC_SCL_H();					//SCL_H
	delay_us(IIC_DELAY);
	SoftwareIIC_SDA_L();					//SDA_L
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_L();					//SCL_L
}
void SoftwareIIC::Stop(void)
{   
	sda_out();
	SoftwareIIC_SCL_L(); 					//SCL_L
	SoftwareIIC_SDA_L();					//SDA_L
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_H();					//SCL_H
	SoftwareIIC_SDA_H();	 				//SDA_H
	delay_us(IIC_DELAY);
}
//����ԭ��:		uint8_t SoftwareIIC::Read_Ack(void)
// �� ��:	    ��ȡACK
uint8_t SoftwareIIC::Read_Ack(void)
{
    uint8_t ReAck;
	sda_in();
	SoftwareIIC_SCL_H();					//SCL_H	
	delay_us(IIC_DELAY);
	ReAck=(uint8_t)SoftwareIIC_SDA_Read();  //Read SDA
	SoftwareIIC_SCL_L();					//SCL_L
	delay_us(IIC_DELAY);
	return ReAck;
}
//����ԭ��:		void SoftwareIIC::Write_OneByte(uint8_t Dat)
//��������:	    д�롢����һ���ֽڵ�����
//����	Dat     д�������
void SoftwareIIC::Write_OneByte(uint8_t Dat)
{
	uint8_t i;  
	sda_out();
	for(i=0;i<8;i++)
	{  
		SoftwareIIC_SCL_L();					//SCL_L
		delay_us(IIC_DELAY);
		if(Dat&0x80)
		{
			SoftwareIIC_SDA_H();				//SDA_H
		}
		else
		{
			SoftwareIIC_SDA_L();				//SDA_L
		}
		Dat<<=1;
		SoftwareIIC_SCL_H();					//SCL_H
		delay_us(IIC_DELAY);
		SoftwareIIC_SCL_L();					//SCL_L
		delay_us(IIC_DELAY);
	}
	Read_Ack();
}
//����ԭ��:		uint8_t SoftwareIIC::Read_OneByte(void)
//��������:	    ��ȡ������һ���ֽڵ�����
uint8_t SoftwareIIC::Read_OneByte(void)
{  
    uint8_t data,i;
	SoftwareIIC_SDA_H();						//SDA_H
	delay_us(IIC_DELAY);
    for(i=0;i<8;i++)
    {
        data<<=1;
		SoftwareIIC_SCL_L();					//SCL_L
        delay_us(IIC_DELAY);
		SoftwareIIC_SCL_H();					//SCL_H
        delay_us(IIC_DELAY);
        if(SoftwareIIC_SDA_Read())  //Read SDA
            data=data | 0x01;
        else 
            data=data & 0xFE;
    }
	SoftwareIIC_SCL_L();						//SCL_L
	delay_us(IIC_DELAY);
    return data;
}
//����ԭ��:		bool SoftwareIIC::get_ack() 
//��������:	    ��ȡACK
bool SoftwareIIC::get_ack() 
{   
	SoftwareIIC_SCL_L();						//SCL_L
	delay_us(IIC_DELAY);
	SoftwareIIC_SDA_H();						//SDA_H
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_H();						//SCL_H
	bool ret = !sda_read();
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_L();						//SCL_L
	return ret;
}

//����ԭ��:		void SoftwareIIC::send_ack() 
//��������:	    ����ACK
void SoftwareIIC::send_ack() 
{   
	SoftwareIIC_SDA_L();						//SDA_L
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_H();						//SCL_H
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_L();						//SCL_L
}
//����ԭ��:		void SoftwareIIC::send_nack() 
//��������:	    ����NACK
void SoftwareIIC::send_nack() 
{  
	SoftwareIIC_SDA_H();						//SDA_H
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_H();						//SCL_H
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_L();						//SCL_L
}
//����ԭ��:		uint8_t SoftwareIIC::shift_in() 
//��������:	    ��ȡ����������
uint8_t SoftwareIIC::shift_in() 
{
    uint8_t data = 0;
    SoftwareIIC_SDA_H();						//SDA_H
    int i;
    for (i = 0; i < 8; i++) 
	{   
		delay_us(IIC_DELAY);
		SoftwareIIC_SCL_H();					//SCL_H
        data |= sda_read() << (7-i);
		delay_us(IIC_DELAY);
        SoftwareIIC_SCL_L();					//SCL_L
    }
    return data;
}
//����ԭ��:		uint8_t SoftwareIIC::shift_in() 
//��������:	    ���͡�д������
void SoftwareIIC::shift_out(uint8_t val) 
{
    int i;
    for (i = 0; i < 8; i++) 
	{
		if(!!(val& (1<<(7-i))) == 0)
		{
			SoftwareIIC_SDA_L();				//SDA_L
		}
		if(!!(val& (1<<(7-i))) == 1)
		{
			SoftwareIIC_SDA_H();				//SDA_H
		}
		delay_us(IIC_DELAY);		
		SoftwareIIC_SCL_H();					//SCL_H
        delay_us(IIC_DELAY);
		SoftwareIIC_SCL_L(); 					//SCL_L
    }
}
//����ԭ��:		uint8_t SoftwareIIC::process()
//��������:	    ���͡�д�� ���ա���ȡ ����ʵ�ֺ���
uint8_t SoftwareIIC::process() 
{
    itc_msg.xferred = 0;
    uint8_t sla_addr = (itc_msg.addr << 1);
    if (itc_msg.flags == I2C_MSG_READ) 
	{
        sla_addr |= I2C_READ;
    }
    Start();
    // shift out the address we're transmitting to
    shift_out(sla_addr);
    if (!get_ack()) 
	{
		Stop();// Roger Clark. 20141110 added to set clock high again, as it will be left in a low state otherwise
        return ENACKADDR;
    }
    // Recieving
    if (itc_msg.flags == I2C_MSG_READ) 
	{
        while (itc_msg.xferred < itc_msg.length) 
		{
            itc_msg.data[itc_msg.xferred++] = shift_in();
            if (itc_msg.xferred < itc_msg.length) 
			{
                send_ack();
            } 
			else 
			{
                send_nack();
            }
        }
    }
    // Sending
    else 
	{
        for (uint8_t i = 0; i < itc_msg.length; i++)
		{
            shift_out(itc_msg.data[i]);
            if (!get_ack()) 
			{
				Stop();// Roger Clark. 20141110 added to set clock high again, as it will be left in a low state otherwise
                return ENACKTRNS;
            }
            itc_msg.xferred++;
        }
    }
    Stop();
    return SUCCESS;
}

//����ԭ��:		void SoftwareIIC::WriteCmd(uint8_t Cmd)
//��������:	    д�롢����ָ��
//����	Cmd     д�롢���͵�ָ��
void SoftwareIIC::WriteCmd(uint8_t Cmd)
{
	Start();
	Write_OneByte(0x78);
	//Read_Ask();
	Write_OneByte(0x00);
	//Read_Ask();
	Write_OneByte(Cmd);
	//Read_Ask();
	Stop();
}
//����ԭ��:		void SoftwareIIC::WriteDat(uint8_t Data)
//��������:	    д�롢��������
//����	Cmd     д�롢���͵�����
void SoftwareIIC::WriteDat(uint8_t Data)
{
	Start();
	Write_OneByte(0x78);
	//Read_Ask();
	Write_OneByte(0x40);
	//Read_Ask();
	Write_OneByte(Data);
	//Read_Ask();
	Stop();
}
//����ԭ��:		int SoftwareIIC::Write_Addr_Data(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
//��������:	    ��ָ���ĵ�ַд�롢��������һ�����ȵ��ַ���������
int SoftwareIIC::Write_Addr_Data(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
    Write_OneByte(addr << 1 );
    if (!Wait_Ack_Reply()) 
	{
        Stop();
        return 1;
    }
    Write_OneByte(reg);
    Wait_Ack_Reply();
	for (i = 0; i < len; i++) 
	{
        Write_OneByte(data[i]);
        if (!Wait_Ack_Reply()) 
		{
            Stop();
            return 0;
        }
    }
    Stop();
    return 0;

}
//����ԭ��:		int SoftwareIIC::Read_Addr_Data(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
//��������:	    ��ȡ������ָ����ַһ�����ȵ��ַ���������
int SoftwareIIC::Read_Addr_Data(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    Write_OneByte(addr << 1);
    if (!Wait_Ack_Reply()) 
	{
        Stop();
        return 1;
    }
    Write_OneByte(reg);
    Wait_Ack_Reply();
    Start();
    Write_OneByte((addr << 1)+1);
    Wait_Ack_Reply();
    while (len) {
        if (len == 1)
            *buf = Read_Byte_Ack(0);
        else
            *buf = Read_Byte_Ack(1);
        buf++;
        len--;
    }
    Stop();
    return 0;
}
//����ԭ��:		uint8_t SoftwareIIC::Read_DeviceAddr_OneByte(uint8_t Addr,uint8_t addr)
//��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
//����	Addr    Ŀ���豸��ַ
//		addr	�Ĵ�����ַ
//����   ��������ֵ
uint8_t SoftwareIIC::Read_DeviceAddr_OneByte(uint8_t Addr,uint8_t addr)
{
	uint8_t res=0;
	Start();	
	Write_OneByte(Addr);	   //����д����
	res++;
	Wait_Ack_Reply();
	Write_OneByte(addr); res++;  //���͵�ַ
	Wait_Ack_Reply();	  
	//IIC_Stop();//����һ��ֹͣ����	
	Start();
	Write_OneByte(Addr+1); res++;          //�������ģʽ			   
	Wait_Ack_Reply();
	res=Read_Byte_Ack(0);	   
    Stop();//����һ��ֹͣ����
	return res;
}
//����ԭ��:		int SoftwareIIC::Wait_Ack_Reply(void)
// �� ��:	    �ȴ�ACKӦ��
//����  int:  	0 ��ACKӦ��  1��ACKӦ��
int SoftwareIIC::Wait_Ack_Reply(void)
{
	uint8_t ucErrTime=0;
	sda_in();      						//SDA Set Input
	SoftwareIIC_SDA_H();	 			//SDA_H
	delay_us(IIC_DELAY2);	   
	SoftwareIIC_SCL_H();				//SCL_H
	delay_us(IIC_DELAY2);	 
	while(sda_read())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Stop();
			return 1;
		}
	}
	SoftwareIIC_SCL_L();				//SCL_L  
	return 0; 
} 
//����ԭ��:		void SoftwareIIC::Generate_Ack_Reply(void)
// �� ��:	    ����ACKӦ��
void SoftwareIIC::Generate_Ack_Reply(void)
{   
	SoftwareIIC_SCL_L(); 				//SCL_L
	sda_out();                          //SDA_OUT
	SoftwareIIC_SDA_L();				//SDA_L
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_H();	 			//SCL_H
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_L(); 				//SCL_L	
}
//����ԭ��:		void SoftwareIIC::NoGenerate_Ack_Reply(void)
// �� ��:	    ������ACKӦ��
void  SoftwareIIC::NoGenerate_Ack_Reply(void)
{   
	SoftwareIIC_SCL_L(); 				//SCL_L
	sda_out();                          //SDA_OUT
	SoftwareIIC_SDA_H();				//SDA_H
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_H();	 			//SCL_H
	delay_us(IIC_DELAY);
	SoftwareIIC_SCL_L(); 				//SCL_L	
}
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void SoftwareIIC::Send_Byte(uint8_t txd)
{    
    uint8_t t;   
	sda_out();							//SDA����Ϊ���	    
    SoftwareIIC_SCL_L();//SCL_L ����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        if(((txd&0x80)>>7) == 0)
		{
			SoftwareIIC_SDA_L();		//SDA_L
		}
		if(((txd&0x80)>>7) == 1)
		{
			SoftwareIIC_SDA_H();		//SDA_H
		}
        txd<<=1; 	  
		delay_us(IIC_DELAY2);   //��TEA5767��������ʱ���Ǳ����
		SoftwareIIC_SCL_H();			//SCL_H IIC_SCL=1;
		delay_us(IIC_DELAY2); 
		SoftwareIIC_SCL_L(); 			//SCL_L IIC_SCL=0;	
		delay_us(IIC_DELAY2);
    }	 
} 	 
//����ԭ��:		uint8_t SoftwareIIC::Read_Byte_Ack(uint8_t ack)
//��������:	    ��ȡ������һ���ֽ����� ACKӦ��ʽ
uint8_t SoftwareIIC::Read_Byte_Ack(uint8_t ack)
{
	uint8_t i,receive=0;
	sda_in();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
		SoftwareIIC_SCL_L(); 			//SCL_L
		delay_us(IIC_DELAY2);
		SoftwareIIC_SCL_H();			//SCL_H
		receive<<=1;
		if(sda_read())
			receive++;
		delay_us(IIC_DELAY2);
    }					 
    if (!ack)
		NoGenerate_Ack_Reply();			//����nACK  
    else
        Generate_Ack_Reply(); 			//����ACK 
    return receive;
}

SoftwareIIC::~SoftwareIIC()
{

}




