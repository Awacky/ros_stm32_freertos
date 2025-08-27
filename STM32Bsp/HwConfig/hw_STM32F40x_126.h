#ifndef _HW_STM32F40x_H_
#define _HW_STM32F40x_H_
//SYSCLK（系统时钟） =168MHz
//AHB 总线时钟(HCLK=SYSCLK) =168MHz
//APB1 总线时钟(PCLK1=SYSCLK/4) =42MHz
//APB2 总线时钟(PCLK2=SYSCLK/2) =84MHz
//PLL 主时钟 =168MHz

#include "stm32f4xx.h" 
#include "datatypes.h"
#include "modem.h"
#include "delay.h"

#define CHIP_UUID				((uint32_t*)0x1FFF7A10)
#define CHIP_UUID_8				((uint8_t*)0x1FFF7A10)
#define FLASH_SIZE_ADDR			((uint32_t*)0x1FFF7A22)
#define HARDWARE_VER            "STF407VG-V1.2.6"
#define Hw_GPIO_TypeDef			GPIO_TypeDef
#define Hw_TIM_TypeDef			TIM_TypeDef
#define Hw_CLK_TypeDef			uint32_t
#define Hw_TCLK_TypeDef			uint32_t
#define Hw_TIM_CHA_TypeDef		uint16_t
#define Hw_GPIO_Set				BSRRL
#define Hw_GPIO_ReSet			BSRRH

/** --------Serial Config-------- **/
#ifndef BOOTLOADER
#define SERIAL1_CONFIG
#define SERIAL2_CONFIG
#define SERIAL3_CONFIG
#define SERIAL4_CONFIG
#define SERIAL5_CONFIG
#endif

#define LEDRn						    	3
#define LEDPWM_CNT_TOP						200
#define STARBOT_LED_STATUS_PORT				GPIOA
#define STARBOT_LED_STATUS_CLK				RCC_AHB1Periph_GPIOA
#define STARBOT_LED_STATUS_PIN				GPIO_Pin_3
#define STARBOT_LED_STATUS_On()             GPIO_ResetBits(STARBOT_LED_STATUS_PORT,STARBOT_LED_STATUS_PIN)          
#define STARBOT_LED_STATUS_Off()            GPIO_SetBits(STARBOT_LED_STATUS_PORT,STARBOT_LED_STATUS_PIN)  
#define STARBOT_LED_STATUS_Toggle()         GPIO_ToggleBits(STARBOT_LED_STATUS_PORT, STARBOT_LED_STATUS_PIN)

#define STARBOT_LED_RUN_PORT				GPIOC
#define STARBOT_LED_RUN_CLK					RCC_AHB1Periph_GPIOC
#define STARBOT_LED_RUN_PIN					GPIO_Pin_1
#define STARBOT_LED_RUN_On()             	GPIO_ResetBits(STARBOT_LED_RUN_PORT,STARBOT_LED_RUN_PIN)        
#define STARBOT_LED_RUN_Off()            	GPIO_SetBits(STARBOT_LED_RUN_PORT,STARBOT_LED_RUN_PIN)  
#define STARBOT_LED_RUN_Toggle()         	GPIO_ToggleBits(STARBOT_LED_RUN_PORT,STARBOT_LED_RUN_PIN)

#define STARBOT_LED_FAULT_PORT				GPIOA
#define STARBOT_LED_FAULT_CLK				RCC_AHB1Periph_GPIOA
#define STARBOT_LED_FAULT_PIN				GPIO_Pin_4
#define STARBOT_LED_FAULT_On()             	GPIO_ResetBits(STARBOT_LED_FAULT_PORT,STARBOT_LED_FAULT_PIN)         
#define STARBOT_LED_FAULT_Off()            	GPIO_SetBits(STARBOT_LED_FAULT_PORT,STARBOT_LED_FAULT_PIN) 
#define STARBOT_LED_FAULT_Toggle()         	GPIO_ToggleBits(STARBOT_LED_FAULT_PORT,STARBOT_LED_FAULT_PIN)

#define STARBOT_BEEP_PIN					GPIO_Pin_2
#define STARBOT_BEEP_PORT					GPIOE
#define STARBOT_BEEP_CLK					RCC_AHB1Periph_GPIOE
#define STARBOT_BEEP_On()					GPIO_SetBits(STARBOT_BEEP_PORT,STARBOT_BEEP_PIN)
#define STARBOT_BEEP_Off()					GPIO_ResetBits(STARBOT_BEEP_PORT,STARBOT_BEEP_PIN)
#define STARBOT_BEEP_Toggle()				GPIO_ToggleBits(STARBOT_BEEP_PORT,STARBOT_BEEP_PIN)


#define POWER_KEYn 							3
#define STARBOT_POWER_KEY1_PORT				GPIOD
#define STARBOT_POWER_KEY1_CLK    			RCC_AHB1Periph_GPIOD
#define STARBOT_POWER_KEY1_PIN				GPIO_Pin_10 
#define STARBOT_POWER_KEY1_On()				GPIO_SetBits(STARBOT_POWER_KEY1_PORT,STARBOT_POWER_KEY1_PIN)
#define STARBOT_POWER_KEY1_Off()			GPIO_ResetBits(STARBOT_POWER_KEY1_PORT,STARBOT_POWER_KEY1_PIN)
#define STARBOT_POWER_KEY2_PORT				GPIOD
#define STARBOT_POWER_KEY2_CLK    			RCC_AHB1Periph_GPIOD
#define STARBOT_POWER_KEY2_PIN				GPIO_Pin_11
#define STARBOT_POWER_KEY2_On()				GPIO_SetBits(STARBOT_POWER_KEY2_PORT,STARBOT_POWER_KEY2_PIN)
#define STARBOT_POWER_KEY2_Off()			GPIO_ResetBits(STARBOT_POWER_KEY2_PORT,STARBOT_POWER_KEY2_PIN)
#define STARBOT_POWER_KEY3_PORT				GPIOD
#define STARBOT_POWER_KEY3_CLK    			RCC_AHB1Periph_GPIOD
#define STARBOT_POWER_KEY3_PIN				GPIO_Pin_14
#define STARBOT_POWER_KEY3_On()				GPIO_SetBits(STARBOT_POWER_KEY3_PORT,STARBOT_POWER_KEY3_PIN)
#define STARBOT_POWER_KEY3_Off()			GPIO_ResetBits(STARBOT_POWER_KEY3_PORT,STARBOT_POWER_KEY3_PIN)

#define STARBOT_EMERGENCY_STOP_PORT			GPIOD
#define STARBOT_EMERGENCY_STOP_CLK    		RCC_AHB1Periph_GPIOD
#define STARBOT_EMERGENCY_STOP_PIN			GPIO_Pin_3  
#define STARBOT_EMERGENCY_Check()           PDin(3)

#define IICn 								3
#define SoftwareII_PORT_TypeDef				GPIO_TypeDef
#define SoftwareII_CLK_TypeDef				uint32_t
#define SoftwareIIC_SDA_In()				GPIO_InitTypeDef GPIO_InitStructer;\
											RCC_AHB1PeriphClockCmd(IIC_SDA_PORT_CLK[this->iic], ENABLE);\
											GPIO_InitStructer.GPIO_Pin = IIC_SDA_PIN[this->iic];\
											GPIO_InitStructer.GPIO_Speed=GPIO_Speed_50MHz;\
											GPIO_InitStructer.GPIO_Mode=GPIO_Mode_IN;\
											GPIO_InitStructer.GPIO_OType = GPIO_OType_OD;\
											GPIO_Init(IIC_SDA_PORT[this->iic], &GPIO_InitStructer);
#define SoftwareIIC_SDA_Out()				GPIO_InitTypeDef GPIO_InitStructer;\
											RCC_AHB1PeriphClockCmd(IIC_SDA_PORT_CLK[this->iic], ENABLE);\
											GPIO_InitStructer.GPIO_Pin = IIC_SDA_PIN[this->iic];\
											GPIO_InitStructer.GPIO_Speed=GPIO_Speed_50MHz;\
											GPIO_InitStructer.GPIO_Mode=GPIO_Mode_OUT;\
											GPIO_InitStructer.GPIO_OType = GPIO_OType_OD;\
											GPIO_Init(IIC_SDA_PORT[this->iic], &GPIO_InitStructer);
#define SoftwareIIC_SDA_H()					IIC_SDA_PORT[this->iic]->BSRRL = IIC_SDA_PIN[this->iic];
#define SoftwareIIC_SDA_L()					IIC_SDA_PORT[this->iic]->BSRRH = IIC_SDA_PIN[this->iic];
#define SoftwareIIC_SCL_H()					IIC_SCL_PORT[this->iic]->BSRRL = IIC_SCL_PIN[this->iic];
#define SoftwareIIC_SCL_L()					IIC_SCL_PORT[this->iic]->BSRRH = IIC_SCL_PIN[this->iic];
#define SoftwareIIC_SDA_Read()				IIC_SDA_PORT[this->iic]->IDR&IIC_SDA_PIN[this->iic]

#define STARBOT_IMU_N_SCL_PIN				GPIO_Pin_7
#define STARBOT_IMU_N_SDA_PIN				GPIO_Pin_4
#define STARBOT_IMU_N_SCL_PORT				GPIOB
#define STARBOT_IMU_N_SDA_PORT				GPIOD
#define STARBOT_IMU_N_SCL_CLK    			RCC_AHB1Periph_GPIOB
#define STARBOT_IMU_N_SDA_CLK    			RCC_AHB1Periph_GPIOD
         

#define STARBOT_IMU_W_SCL_PIN				GPIO_Pin_7
#define STARBOT_IMU_W_SDA_PIN				GPIO_Pin_4
#define STARBOT_IMU_W_SCL_PORT				GPIOD
#define STARBOT_IMU_W_SDA_PORT				GPIOB
#define STARBOT_IMU_W_SCL_CLK    			RCC_AHB1Periph_GPIOD
#define STARBOT_IMU_W_SDA_CLK    			RCC_AHB1Periph_GPIOB


#define STARBOT_OLED_SCL_PIN				GPIO_Pin_1
#define STARBOT_OLED_SDA_PIN				GPIO_Pin_0
#define STARBOT_OLED_SCL_PORT				GPIOD
#define STARBOT_OLED_SDA_PORT				GPIOD
#define STARBOT_OLED_SCL_CLK    			RCC_AHB1Periph_GPIOD
#define STARBOT_OLED_SDA_CLK    			RCC_AHB1Periph_GPIOD

/** PSTWO Config **/
//平的为底面，灯面对自己左到右是9-1
//接收器管脚      板子               接板子丝印
//CLK 7          PE15    SPI-SCL     CLK
//CS  6          PB10     CS         CS
//VDD 5                              VCC（不能5V供电！！）
//GND 4                              GND
//DO  2          PB11    SPI-MOSI    DI 
//DI  1          PE10    SPI-MISO    D0
#define STARBOT_PSTWO_CS_PORT				GPIOB
#define STARBOT_PSTWO_CS_CLK    			RCC_AHB1Periph_GPIOB
#define STARBOT_PSTWO_CS_PIN				GPIO_Pin_10         

#define STARBOT_PSTWO_SCL_PORT				GPIOE
#define STARBOT_PSTWO_SCL_CLK    			RCC_AHB1Periph_GPIOE
#define STARBOT_PSTWO_SCL_PIN				GPIO_Pin_15  

#define STARBOT_PSTWO_DI_PORT				GPIOE
#define STARBOT_PSTWO_DI_CLK    			RCC_AHB1Periph_GPIOE
#define STARBOT_PSTWO_DI_PIN				GPIO_Pin_10  

#define STARBOT_PSTWO_DO_PORT				GPIOB
#define STARBOT_PSTWO_DO_CLK    			RCC_AHB1Periph_GPIOB
#define STARBOT_PSTWO_DO_PIN				GPIO_Pin_11  

#define STARBOT_PSTWO_CHECK_PORT			GPIOE
#define STARBOT_PSTWO_CHECK_CLK    			RCC_AHB1Periph_GPIOE
#define STARBOT_PSTWO_CHECK_PIN				GPIO_Pin_12
#define STARBOT_PSTWO_Check()               PEin(12)

#define DI    								PEin(10)      //PE10输入
#define DO_H  								PBout(11)=1   //命令位高
#define DO_L  								PBout(11)=0   //命令位低
#define CS_H  								PBout(10)=1   //CS拉高
#define CS_L  								PBout(10)=0   //CS拉低
#define CLK_H 								PEout(15)=1   //时钟拉高
#define CLK_L 								PEout(15)=0   //时钟拉低

/** -----------Motor Config ---------**/ 
#define MOTORn						        4
#define STARBOT_MOTOR1_A_PIN		 		GPIO_Pin_13
#define STARBOT_MOTOR1_A_PINSOU		 		GPIO_PinSource13
#define STARBOT_MOTOR1_A_GPIO_PORT	 		GPIOE
#define STARBOT_MOTOR1_A_GPIO_CLK	 		RCC_AHB1Periph_GPIOE
#define STARBOT_MOTOR1_B_PIN   	 	   	 	GPIO_Pin_11
#define STARBOT_MOTOR1_B_PINSOU		 		GPIO_PinSource11
#define STARBOT_MOTOR1_B_GPIO_PORT	 		GPIOE
#define STARBOT_MOTOR1_B_GPIO_CLK	 		RCC_AHB1Periph_GPIOE
#define STARBOT_MOTOR1_TIM            		TIM1
#define STARBOT_MOTOR1_TIM_CLK        		RCC_APB2Periph_TIM1
#define STARBOT_MOTOR1_A_CHANNEL			TIM_Channel_3
#define STARBOT_MOTOR1_B_CHANNEL			TIM_Channel_2
#define STARBOT_MOTOR1_AF_TIM               GPIO_AF_TIM1

#define STARBOT_MOTOR2_A_PIN		 		GPIO_Pin_14
#define STARBOT_MOTOR2_A_PINSOU		 		GPIO_PinSource14
#define STARBOT_MOTOR2_A_GPIO_PORT	 		GPIOE
#define STARBOT_MOTOR2_A_GPIO_CLK	 		RCC_AHB1Periph_GPIOE
#define STARBOT_MOTOR2_B_PIN   	 	   	 	GPIO_Pin_9
#define STARBOT_MOTOR2_B_PINSOU		 		GPIO_PinSource9
#define STARBOT_MOTOR2_B_GPIO_PORT	 		GPIOE
#define STARBOT_MOTOR2_B_GPIO_CLK	 		RCC_AHB1Periph_GPIOE
#define STARBOT_MOTOR2_TIM            		TIM1
#define STARBOT_MOTOR2_TIM_CLK        		RCC_APB2Periph_TIM1
#define STARBOT_MOTOR2_A_CHANNEL			TIM_Channel_4
#define STARBOT_MOTOR2_B_CHANNEL			TIM_Channel_1
#define STARBOT_MOTOR2_AF_TIM               GPIO_AF_TIM1

#define STARBOT_MOTOR3_A_PIN		 		GPIO_Pin_6
#define STARBOT_MOTOR3_A_PINSOU		 		GPIO_PinSource6
#define STARBOT_MOTOR3_A_GPIO_PORT	 		GPIOC
#define STARBOT_MOTOR3_A_GPIO_CLK	 		RCC_AHB1Periph_GPIOC
#define STARBOT_MOTOR3_B_PIN   	 	   	 	GPIO_Pin_7
#define STARBOT_MOTOR3_B_PINSOU		 		GPIO_PinSource7
#define STARBOT_MOTOR3_B_GPIO_PORT	 		GPIOC
#define STARBOT_MOTOR3_B_GPIO_CLK	 		RCC_AHB1Periph_GPIOC
#define STARBOT_MOTOR3_TIM            		TIM8
#define STARBOT_MOTOR3_TIM_CLK        		RCC_APB2Periph_TIM8
#define STARBOT_MOTOR3_A_CHANNEL			TIM_Channel_1
#define STARBOT_MOTOR3_B_CHANNEL			TIM_Channel_2
#define STARBOT_MOTOR3_AF_TIM               GPIO_AF_TIM8

//PA8 TIM_CH1 TIM_CH1N
#define STARBOT_MOTOR4_A_PIN		 		GPIO_Pin_9
#define STARBOT_MOTOR4_A_PINSOU		 		GPIO_PinSource9
#define STARBOT_MOTOR4_A_GPIO_PORT	 		GPIOC
#define STARBOT_MOTOR4_A_GPIO_CLK	 		RCC_AHB1Periph_GPIOC
#define STARBOT_MOTOR4_B_PIN   	 	   	 	GPIO_Pin_8
#define STARBOT_MOTOR4_B_PINSOU		 		GPIO_PinSource8
#define STARBOT_MOTOR4_B_GPIO_PORT	 		GPIOC
#define STARBOT_MOTOR4_B_GPIO_CLK	 		RCC_AHB1Periph_GPIOC
#define STARBOT_MOTOR4_TIM            		TIM8
#define STARBOT_MOTOR4_TIM_CLK        		RCC_APB2Periph_TIM8
#define STARBOT_MOTOR4_A_CHANNEL			TIM_Channel_4
#define STARBOT_MOTOR4_B_CHANNEL			TIM_Channel_3
#define STARBOT_MOTOR4_AF_TIM               GPIO_AF_TIM8

#define ENCODERn 					        4
#define STARBOT_ENCODER1_A_PIN           	GPIO_Pin_0
#define STARBOT_ENCODER1_A_GPIO_PORT      	GPIOA
#define STARBOT_ENCODER1_A_GPIO_PinSource 	GPIO_PinSource0
#define STARBOT_ENCODER1_A_GPIO_CLK    		RCC_AHB1Periph_GPIOA
#define STARBOT_ENCODER1_B_PIN           	GPIO_Pin_1
#define STARBOT_ENCODER1_B_GPIO_PORT   		GPIOA
#define STARBOT_ENCODER1_B_GPIO_PinSource 	GPIO_PinSource1
#define STARBOT_ENCODER1_B_GPIO_CLK    		RCC_AHB1Periph_GPIOA
#define STARBOT_ENCODER1_TIM				TIM5
#define STARBOT_ENCODER1_TIR				TIM5_IRQn
#define STARBOT_ENCODER1_TIM_CLK 			RCC_APB1Periph_TIM5
#define STARBOT_ENCODER1_GPIO_AF_TIM      	GPIO_AF_TIM5

#define STARBOT_ENCODER2_A_PIN           	GPIO_Pin_5
#define STARBOT_ENCODER2_A_GPIO_PORT      	GPIOA
#define STARBOT_ENCODER2_A_GPIO_PinSource 	GPIO_PinSource5
#define STARBOT_ENCODER2_A_GPIO_CLK    		RCC_AHB1Periph_GPIOA
#define STARBOT_ENCODER2_B_PIN           	GPIO_Pin_3
#define STARBOT_ENCODER2_B_GPIO_PORT   		GPIOB
#define STARBOT_ENCODER2_B_GPIO_PinSource 	GPIO_PinSource3
#define STARBOT_ENCODER2_B_GPIO_CLK    		RCC_AHB1Periph_GPIOB
#define STARBOT_ENCODER2_TIM				TIM2
#define STARBOT_ENCODER2_TIR				TIM2_IRQn
#define STARBOT_ENCODER2_TIM_CLK 			RCC_APB1Periph_TIM2
#define STARBOT_ENCODER2_GPIO_AF_TIM      	GPIO_AF_TIM2

#define STARBOT_ENCODER3_A_PIN           	GPIO_Pin_6
#define STARBOT_ENCODER3_A_GPIO_PORT      	GPIOA
#define STARBOT_ENCODER3_A_GPIO_PinSource 	GPIO_PinSource6
#define STARBOT_ENCODER3_A_GPIO_CLK    		RCC_AHB1Periph_GPIOA
#define STARBOT_ENCODER3_B_PIN           	GPIO_Pin_7
#define STARBOT_ENCODER3_B_GPIO_PORT   		GPIOA
#define STARBOT_ENCODER3_B_GPIO_PinSource 	GPIO_PinSource7
#define STARBOT_ENCODER3_B_GPIO_CLK    		RCC_AHB1Periph_GPIOA
#define STARBOT_ENCODER3_TIM				TIM3
#define STARBOT_ENCODER3_TIR				TIM3_IRQn
#define STARBOT_ENCODER3_TIM_CLK 			RCC_APB1Periph_TIM3
#define STARBOT_ENCODER3_GPIO_AF_TIM      	GPIO_AF_TIM3

#define STARBOT_ENCODER3_A2_PIN           	GPIO_Pin_4
#define STARBOT_ENCODER3_A2_GPIO_PORT      	GPIOB
#define STARBOT_ENCODER3_A2_GPIO_PinSource 	GPIO_PinSource4
#define STARBOT_ENCODER3_A2_GPIO_CLK    	RCC_AHB1Periph_GPIOB
#define STARBOT_ENCODER3_B2_PIN           	GPIO_Pin_5
#define STARBOT_ENCODER3_B2_GPIO_PORT   	GPIOB
#define STARBOT_ENCODER3_B2_GPIO_PinSource 	GPIO_PinSource5
#define STARBOT_ENCODER3_B2_GPIO_CLK    	RCC_AHB1Periph_GPIOB
#define STARBOT_ENCODER3_Z_PIN           	GPIO_Pin_9
#define STARBOT_ENCODER3_Z_GPIO_PORT   		GPIOC
#define STARBOT_ENCODER3_Z_GPIO_PinSource 	GPIO_PinSource9
#define STARBOT_ENCODER3_Z_GPIO_CLK    		RCC_AHB1Periph_GPIOC

#define STARBOT_ENCODER4_A_PIN           	GPIO_Pin_13
#define STARBOT_ENCODER4_A_GPIO_PORT      	GPIOD
#define STARBOT_ENCODER4_A_GPIO_PinSource 	GPIO_PinSource13
#define STARBOT_ENCODER4_A_GPIO_CLK    		RCC_AHB1Periph_GPIOD
#define STARBOT_ENCODER4_B_PIN           	GPIO_Pin_12
#define STARBOT_ENCODER4_B_GPIO_PORT   		GPIOD
#define STARBOT_ENCODER4_B_GPIO_PinSource 	GPIO_PinSource12
#define STARBOT_ENCODER4_B_GPIO_CLK    		RCC_AHB1Periph_GPIOD
#define STARBOT_ENCODER4_TIM				TIM4
#define STARBOT_ENCODER4_TIR				TIM4_IRQn
#define STARBOT_ENCODER4_TIM_CLK 			RCC_APB1Periph_TIM4
#define STARBOT_ENCODER4_GPIO_AF_TIM      	GPIO_AF_TIM4
/** ADC config **/
#define ADC1_DR_ADDRESS           			((u32)ADC1+0x4c)
#define ADC_n 					      		8
#define STARBOT_ADC1_PIN					GPIO_Pin_2
#define STARBOT_ADC1_GPIO_PORT				GPIOA
#define STARBOT_ADC1_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define STARBOT_ADC1_CHANNEL         		ADC_Channel_2

#define STARBOT_ADC2_PIN					GPIO_Pin_0
#define STARBOT_ADC2_GPIO_PORT				GPIOC
#define STARBOT_ADC2_GPIO_CLK				RCC_AHB1Periph_GPIOC
#define STARBOT_ADC2_CHANNEL         		ADC_Channel_10

#define STARBOT_M1CurrH_PIN					GPIO_Pin_6
#define STARBOT_M1CurrH_GPIO_PORT			GPIOA
#define STARBOT_M1CurrH_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define STARBOT_M1CurrH_CHANNEL         	ADC_Channel_6

#define STARBOT_M1CurrL_PIN					GPIO_Pin_6
#define STARBOT_M1CurrL_GPIO_PORT			GPIOA
#define STARBOT_M1CurrL_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define STARBOT_M1CurrL_CHANNEL         	ADC_Channel_6

#define STARBOT_M2CurrH_PIN					GPIO_Pin_6
#define STARBOT_M2CurrH_GPIO_PORT			GPIOA
#define STARBOT_M2CurrH_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define STARBOT_M2CurrH_CHANNEL         	ADC_Channel_6

#define STARBOT_M2CurrL_PIN					GPIO_Pin_6
#define STARBOT_M2CurrL_GPIO_PORT			GPIOA
#define STARBOT_M2CurrL_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define STARBOT_M2CurrL_CHANNEL         	ADC_Channel_6

#define STARBOT_M3CurrH_PIN					GPIO_Pin_6
#define STARBOT_M3CurrH_GPIO_PORT			GPIOA
#define STARBOT_M3CurrH_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define STARBOT_M3CurrH_CHANNEL         	ADC_Channel_6

#define STARBOT_M3CurrL_PIN					GPIO_Pin_6
#define STARBOT_M3CurrL_GPIO_PORT			GPIOA
#define STARBOT_M3CurrL_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define STARBOT_M3CurrL_CHANNEL         	ADC_Channel_6

#define STARBOT_M4Curr_PIN					GPIO_Pin_6
#define STARBOT_M4Curr_GPIO_PORT			GPIOA
#define STARBOT_M4Curr_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define STARBOT_M4Curr_CHANNEL         		ADC_Channel_6

/** Sonar config **/
#define SONARn 								2
#define STARBOT_SONAR1_TRIG_PIN				GPIO_Pin_1
#define STARBOT_SONAR1_TRIG_PORT			GPIOE
#define STARBOT_SONAR1_TRIG_CLK				RCC_AHB1Periph_GPIOE
#define STARBOT_SONAR1_ECHO_PIN				GPIO_Pin_9
#define STARBOT_SONAR1_ECHO_PORT			GPIOB
#define STARBOT_SONAR1_ECHO_CLK				RCC_AHB1Periph_GPIOB
#define STARBOT_SONAR1_PINSOURCE			GPIO_PinSource9
#define STARBOT_SONAR1_TIM					TIM11
#define STARBOT_SONAR1_TIM_CLK				RCC_APB2Periph_TIM11
#define STARBOT_SONAR1_AF_TIM				GPIO_AF_TIM11
#define STARBOT_SONAR1_TIM_IRQ				TIM1_TRG_COM_TIM11_IRQn
#define STARBOT_SONAR1_TIM_PSC				167
#define STARBOT_SONAR1_TIM_ARR				0XFFFF
#define STARBOT_SONAR1_TRIG_ON()			GPIO_SetBits(STARBOT_SONAR1_TRIG_PORT,STARBOT_SONAR1_TRIG_PIN)
#define STARBOT_SONAR1_TRIG_OFF()			GPIO_ResetBits(STARBOT_SONAR1_TRIG_PORT,STARBOT_SONAR1_TRIG_PIN)

#define STARBOT_SONAR2_TRIG_PIN				GPIO_Pin_0
#define STARBOT_SONAR2_TRIG_PORT			GPIOE
#define STARBOT_SONAR2_TRIG_CLK				RCC_AHB1Periph_GPIOE
#define STARBOT_SONAR2_ECHO_PIN				GPIO_Pin_8
#define STARBOT_SONAR2_ECHO_PORT			GPIOB
#define STARBOT_SONAR2_ECHO_CLK				RCC_AHB1Periph_GPIOB
#define STARBOT_SONAR2_PINSOURCE			GPIO_PinSource8
#define STARBOT_SONAR2_TIM					TIM10
#define STARBOT_SONAR2_TIM_CLK				RCC_APB2Periph_TIM10
#define STARBOT_SONAR2_AF_TIM				GPIO_AF_TIM10
#define STARBOT_SONAR2_TIM_IRQ				TIM1_UP_TIM10_IRQn
#define STARBOT_SONAR2_TIM_PSC				167
#define STARBOT_SONAR2_TIM_ARR				0XFFFF
#define STARBOT_SONAR2_TRIG_ON()			GPIO_SetBits(STARBOT_SONAR2_TRIG_PORT,STARBOT_SONAR2_TRIG_PIN)
#define STARBOT_SONAR2_TRIG_OFF()			GPIO_ResetBits(STARBOT_SONAR2_TRIG_PORT,STARBOT_SONAR2_TRIG_PIN)
 

#define SERVOn 								4
#define MAX_ANGLE							270
#define STARBOT_SERVO1_PIN					GPIO_Pin_6
#define STARBOT_SERVO1_SOUPIN     			GPIO_PinSource6
#define STARBOT_SERVO1_GPIO_PORT			GPIOE
#define STARBOT_SERVO1_TIM					TIM9
#define STARBOT_SERVO1_AFTIM      			GPIO_AF_TIM9
#define STARBOT_SERVO1_CHANNEL				TIM_Channel_2
#define STARBOT_SERVO1_GPIO_CLK				RCC_AHB1Periph_GPIOE
#define STARBOT_SERVO1_TIM_CLK				RCC_APB2Periph_TIM9


#define STARBOT_SERVO2_PIN					GPIO_Pin_5
#define STARBOT_SERVO2_SOUPIN     			GPIO_PinSource5 
#define STARBOT_SERVO2_GPIO_PORT			GPIOE
#define STARBOT_SERVO2_TIM					TIM9
#define STARBOT_SERVO2_AFTIM        		GPIO_AF_TIM9
#define STARBOT_SERVO2_CHANNEL				TIM_Channel_1
#define STARBOT_SERVO2_GPIO_CLK				RCC_AHB1Periph_GPIOE
#define STARBOT_SERVO2_TIM_CLK				RCC_APB2Periph_TIM9

#define STARBOT_SERVO3_PIN					GPIO_Pin_14
#define STARBOT_SERVO3_SOUPIN     			GPIO_PinSource14
#define STARBOT_SERVO3_GPIO_PORT  			GPIOB
#define STARBOT_SERVO3_TIM					TIM12
#define STARBOT_SERVO3_AFTIM        		GPIO_AF_TIM12
#define STARBOT_SERVO3_CHANNEL				TIM_Channel_1
#define STARBOT_SERVO3_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define STARBOT_SERVO3_TIM_CLK				RCC_APB1Periph_TIM12 

#define STARBOT_SERVO4_PIN					GPIO_Pin_15
#define STARBOT_SERVO4_SOUPIN     			GPIO_PinSource15
#define STARBOT_SERVO4_GPIO_PORT  			GPIOB
#define STARBOT_SERVO4_TIM					TIM12
#define STARBOT_SERVO4_AFTIM      			GPIO_AF_TIM12
#define STARBOT_SERVO4_CHANNEL				TIM_Channel_2
#define STARBOT_SERVO4_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define STARBOT_SERVO4_TIM_CLK				RCC_APB1Periph_TIM12

#define STARBOT_SPI_CS_PIN					GPIO_Pin_12
#define STARBOT_SPI_CS_PORT					GPIOB
#define STARBOT_SPI_CS_CLK					RCC_AHB1Periph_GPIOB

#define STARBOT_SPI_MISO_PIN				GPIO_Pin_14
#define STARBOT_SPI_MISO_PS					GPIO_PinSource14
#define STARBOT_SPI_MISO_PORT				GPIOB
#define STARBOT_SPI_MISO_CLK				RCC_AHB1Periph_GPIOB

#define STARBOT_SPI_MOSI_PIN				GPIO_Pin_15
#define STARBOT_SPI_MOSI_PS					GPIO_PinSource15
#define STARBOT_SPI_MOSI_PORT				GPIOB
#define STARBOT_SPI_MOSI_CLK				RCC_AHB1Periph_GPIOB

#define STARBOT_SPI_SCLK_PIN				GPIO_Pin_13
#define STARBOT_SPI_SCLK_PS					GPIO_PinSource13
#define STARBOT_SPI_SCLK_PORT				GPIOB
#define STARBOT_SPI_SCLK_CLK				RCC_AHB1Periph_GPIOB

#define STARBOT_SPI							SPI2
#define STARBOT_SPI_G_AF	 				GPIO_AF_SPI2
#define STARBOT_SPI_CLK						RCC_APB1Periph_SPI2

#define WS2812LEDn							50
#define WS2812n 							2
#define STARBOT_WS2812_1_GPIO_PORT	 		GPIOA
#define STARBOT_WS2812_1_TIM            	TIM3
#define STARBOT_WS2812_1_DMA_Stream 		DMA1_Stream4
#define STARBOT_WS2812_1_PIN		 		GPIO_Pin_6
#define STARBOT_WS2812_1_PINSOU		 		GPIO_PinSource6
#define STARBOT_WS2812_1_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define STARBOT_WS2812_1_TIM_CLK        	RCC_APB1Periph_TIM3
#define STARBOT_WS2812_1_AF_TIM         	GPIO_AF_TIM3
#define STARBOT_WS2812_1_DMA_CLK        	RCC_AHB1Periph_DMA1
#define STARBOT_WS2812_1_DMA_CHANNEL		DMA_Channel_5
#define STARBOT_WS2812_1_TIM_DMA_CC 		TIM_DMA_CC1
#define STARBOT_WS2812_1_DMA_TCIF			DMA_FLAG_TCIF4

#define STARBOT_WS2812_2_GPIO_PORT	 		GPIOD
#define STARBOT_WS2812_2_TIM            	TIM4
#define STARBOT_WS2812_2_DMA_Stream 		DMA1_Stream0
#define STARBOT_WS2812_2_PIN		 		GPIO_Pin_12
#define STARBOT_WS2812_2_PINSOU		 		GPIO_PinSource12
#define STARBOT_WS2812_2_GPIO_CLK			RCC_AHB1Periph_GPIOD
#define STARBOT_WS2812_2_TIM_CLK        	RCC_APB1Periph_TIM4
#define STARBOT_WS2812_2_AF_TIM         	GPIO_AF_TIM4
#define STARBOT_WS2812_2_DMA_CLK        	RCC_AHB1Periph_DMA1
#define STARBOT_WS2812_2_DMA_CHANNEL		DMA_Channel_2
#define STARBOT_WS2812_2_TIM_DMA_CC 		TIM_DMA_CC1
#define STARBOT_WS2812_2_DMA_TCIF			DMA_FLAG_TCIF0 

#define STARBOT_CAN							CAN2
#define STARBOT_CAN_AF						GPIO_AF_CAN2
#define STARBOT_CAN_CLK						RCC_APB1Periph_CAN2
#define STARBOT_CAN_IRQ	 					CAN2_RX0_IRQn
#define STARBOT_CAN_RX_IRQHandler			CAN2_RX0_IRQHandler
#define STARBOT_CAN_FilterNumber			14
#define STARBOT_CAN_TX_CLK					RCC_AHB1Periph_GPIOB
#define STARBOT_CAN_TX_GPIO_PORT			GPIOB
#define STARBOT_CAN_TX_GPIO					GPIO_Pin_5
#define STARBOT_CAN_TX_PINSOURCE			GPIO_PinSource5
#define STARBOT_CAN_RX_CLK					RCC_AHB1Periph_GPIOB
#define STARBOT_CAN_RX_GPIO_PORT			GPIOB
#define STARBOT_CAN_RX_GPIO					GPIO_Pin_6
#define STARBOT_CAN_RX_PINSOURCE			GPIO_PinSource6

/** PPM config **/
#define DEC_DUTY_CH1_PIN					GPIO_Pin_15
#define DEC_DUTY_CH1_PORT					GPIOB
#define DEC_DUTY_CH1_CLK					RCC_AHB1Periph_GPIOB
#define DEC_DUTY_CH1_PINSOURCE				GPIO_PinSource15
#define DEC_DUTY_CH2_PIN					GPIO_Pin_14
#define DEC_DUTY_CH2_PORT					GPIOB
#define DEC_DUTY_CH2_CLK					RCC_AHB1Periph_GPIOB
#define DEC_DUTY_CH2_PINSOURCE				GPIO_PinSource14
#define DEC_DUTY_TIM						TIM12
#define DEC_DUTY_TIM_CLK					RCC_APB1Periph_TIM12
#define DEC_DUTY_AF_TIM						GPIO_AF_TIM12
#define DEC_DUTY_TIM_IRQ					TIM8_BRK_TIM12_IRQn

#define DEC_DUTY_TIM_PSC					83
#define DEC_DUTY_TIM_ARR					0xFFFF

/** PPM config **/
#define DEC_PPM_PIN							GPIO_Pin_15
#define DEC_PPM_PORT						GPIOB
#define DEC_PPM_CLK							RCC_AHB1Periph_GPIOB
#define DEC_PPM_PINSOURCE					GPIO_PinSource15
#define DEC_PPM_TIM							TIM12
#define DEC_PPM_TIM_CLK						RCC_APB1Periph_TIM12
#define DEC_PPM_AF_TIM						GPIO_AF_TIM12
#define DEC_PPM_TIM_IRQ						TIM8_BRK_TIM12_IRQn

#define DEC_PPM_TIM_PSC						83
#define DEC_PPM_TIM_ARR						21000

///////////////////////DCM MOSFET////////////////////////////////

#define HW_M1UpH_GPIO						GPIOE
#define HW_M1UpL_GPIO						GPIOE
#define HW_M1DowH_GPIO						GPIOE
#define HW_M1DowL_GPIO						GPIOE
#define HW_M1UpH_PIN						GPIO_Pin_9				//PE9  TIM1_CH1
#define HW_M1UpH_SOURCE						GPIO_PinSource9
#define HW_M1UpL_PIN						GPIO_Pin_8				//PE8  TIM1_CH1N
#define HW_M1UpL_SOURCE						GPIO_PinSource8
#define HW_M1DowH_PIN						GPIO_Pin_11				//PE11 TIM1_CH2
#define HW_M1DowH_SOURCE					GPIO_PinSource11
#define HW_M1DowL_PIN						GPIO_Pin_10				//PE10 TIM1_CH2N
#define HW_M1DowL_SOURCE					GPIO_PinSource10
#define HW_M1UpH_TIM_AF						GPIO_AF_TIM1
#define HW_M1UpL_TIM_AF						GPIO_AF_TIM1
#define HW_M1DowH_TIM_AF					GPIO_AF_TIM1
#define HW_M1DowL_TIM_AF					GPIO_AF_TIM1

#define HW_M2UpH_GPIO						GPIOE
#define HW_M2UpL_GPIO						GPIOE
#define HW_M2DowH_GPIO						GPIOC
#define HW_M2DowL_GPIO						GPIOA
#define HW_M2UpH_PIN						GPIO_Pin_13				//PE13  TIM1_CH3
#define HW_M2UpH_SOURCE						GPIO_PinSource13
#define HW_M2UpL_PIN						GPIO_Pin_12				//PE12  TIM1_CH3N
#define HW_M2UpL_SOURCE						GPIO_PinSource12
#define HW_M2DowH_PIN						GPIO_Pin_6				//PC6   TIM8_CH1
#define HW_M2DowH_SOURCE					GPIO_PinSource6
#define HW_M2DowL_PIN						GPIO_Pin_7				//PA7   TIM8_CH1N
#define HW_M2DowL_SOURCE					GPIO_PinSource7
#define HW_M2UpH_TIM_AF						GPIO_AF_TIM1
#define HW_M2UpL_TIM_AF						GPIO_AF_TIM1
#define HW_M2DowH_TIM_AF					GPIO_AF_TIM8
#define HW_M2DowL_TIM_AF					GPIO_AF_TIM8

#define HW_M3UpH_GPIO						GPIOC
#define HW_M3UpL_GPIO						GPIOB
#define HW_M3DowH_GPIO						GPIOC
#define HW_M3DowL_GPIO						GPIOB
#define HW_M3UpH_PIN						GPIO_Pin_7				//PC7  TIM8_CH2
#define HW_M3UpH_SOURCE						GPIO_PinSource7
#define HW_M3UpL_PIN						GPIO_Pin_0				//PB0  TIM8_CH2N
#define HW_M3UpL_SOURCE						GPIO_PinSource0
#define HW_M3DowH_PIN						GPIO_Pin_8				//PC8  TIM8_CH3
#define HW_M3DowH_SOURCE					GPIO_PinSource8
#define HW_M3DowL_PIN						GPIO_Pin_1				//PB1  TIM8_CH3N
#define HW_M3DowL_SOURCE					GPIO_PinSource1
#define HW_M3UpH_TIM_AF						GPIO_AF_TIM8
#define HW_M3UpL_TIM_AF						GPIO_AF_TIM8
#define HW_M3DowH_TIM_AF					GPIO_AF_TIM8
#define HW_M3DowL_TIM_AF					GPIO_AF_TIM8

#define HW_M4UpH_GPIO						GPIOE
#define HW_M4DowH_GPIO						GPIOC
#define HW_M4UpH_PIN						GPIO_Pin_14				//PE14  TIM1_CH4
#define HW_M4UpH_SOURCE						GPIO_PinSource14
#define HW_M4DowH_PIN						GPIO_Pin_9				//PC9   TIM8_CH4
#define HW_M4DowH_SOURCE					GPIO_PinSource9
#define HW_M4UpH_TIM_AF						GPIO_AF_TIM1
#define HW_M4DowH_TIM_AF					GPIO_AF_TIM8
///////////////////////IO REMAP///////////////////////////
#define BITBAND(addr, bitnum) 		((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)              *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   	MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)   
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)   
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)   

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)   
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)   

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)   

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)   

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n) 

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)


#endif // _CONFIG_H_