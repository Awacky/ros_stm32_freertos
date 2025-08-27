#ifdef __cplusplus
extern "C" {
#endif

#include "delay.h"

#if SYSTEM_SUPPORT_OS
	#include "FreeRTOS.h"					//֧��OSʱ��ʹ��	  		  
	#include "task.h"					
#endif

static uint8_t  fac_us=0;							//us��ʱ������			   
static uint16_t fac_ms=0;							//ms��ʱ������,��os��,����ÿ�����ĵ�ms��
static uint32_t sysTickCnt = 0;
#if SYSTEM_SUPPORT_OS
	extern void xPortSysTickHandler(void);				
#endif

//systick�жϷ�����,ʹ��OSʱ�õ�
void SysTick_Handler(void)
{	
	#if SYSTEM_SUPPORT_OS
		if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED){//ϵͳ�Ѿ�����
			xPortSysTickHandler();
		}else{
			sysTickCnt++;
		}
	#else
		sysTickCnt++;
	#endif

}
		   
//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪAHBʱ�ӣ�������������SYSTICKʱ��Ƶ��ΪAHB/8
//����Ϊ�˼���FreeRTOS�����Խ�SYSTICK��ʱ��Ƶ�ʸ�ΪAHB��Ƶ�ʣ�
//SYSCLK:ϵͳʱ��Ƶ��
void delay_init(uint8_t SYSCLK)
{
	#if SYSTEM_SUPPORT_OS 
		uint32_t reload;
	#endif
	#if defined(STM32F40_41xxx) || defined(STM32F10X_HD)
	{
		#ifndef BOOTLOADER
		NVIC_SetVectorTable(APP_ADDR,0);
		#endif
		#ifdef BOOTLOADER
		NVIC_SetVectorTable(BOOT_ADDR,0);
		#endif
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);								//����ϵͳ�ж����ȼ�����4 16
		SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); 							//SysTick ʱ�Ӳ���Ƶ
	}
	#elif  defined(AT32F40x)
	{	
		#ifndef BOOTLOADER
		nvic_vector_table_set(NVIC_VECTTAB_FLASH,APP_ADDR - NVIC_VECTTAB_FLASH);
		#endif
//		#ifdef BOOTLOADER
//		nvic_vector_table_set(NVIC_VECTTAB_FLASH,BOOT_ADDR - NVIC_VECTTAB_FLASH);
//		#endif
		nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);							//����ϵͳ�ж����ȼ�����4 16
		system_clock_config();														//ʱ������
		systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);				//SysTick ʱ�Ӳ���Ƶ
		SYSCLK = 240;
	}
	#endif
	fac_us=SYSCLK;								//�����Ƿ�ʹ��OS,fac_us����Ҫʹ��
	#if SYSTEM_SUPPORT_OS
		reload=SYSCLK;							//ÿ���ӵļ������� ��λΪM	   
		reload*=1000000/configTICK_RATE_HZ;		//����delay_ostickspersec�趨���ʱ��
												//reloadΪ24λ�Ĵ���,���ֵ:16777216,��168M��,Լ��0.0998s����	
		fac_ms=1000/configTICK_RATE_HZ;			//����OS������ʱ�����ٵ�λ	   
		SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//����SYSTICK�ж�
		SysTick->LOAD=reload; 					//ÿ1/configTICK_RATE_HZ��һ��	
		SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //����SYSTICK   
	#else
		fac_ms=(uint16_t)fac_us*1000;				//��OS��,����ÿ��ms��Ҫ��systickʱ����   
	#endif 	
}								    


#if SYSTEM_SUPPORT_OS 						//�����Ҫ֧��OS.
	/********************************************************
	*getSysTickCnt()
	*���ȿ���֮ǰ ���� sysTickCnt
	*���ȿ���֮ǰ ���� xTaskGetTickCount()
	*********************************************************/
	uint32_t getSysTickCnt(void)
	{
		if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*ϵͳ�Ѿ�����*/
			return xTaskGetTickCount();
		else
			return sysTickCnt;
	}	
	//��ʱnus
	//nus:Ҫ��ʱ��us��.	
	//nus:0~204522252(���ֵ��2^32/fac_us@fac_us=168)	    								   
	void delay_us(uint32_t nus)
	{		
		uint32_t ticks;
		uint32_t told,tnow,tcnt=0;
		uint32_t reload=SysTick->LOAD;				//LOAD��ֵ	    	 
		ticks=nus*fac_us; 						//��Ҫ�Ľ����� 
		told=SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
		while(1)
		{
			tnow=SysTick->VAL;	
			if(tnow!=told)
			{	    
				if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
				else tcnt+=reload-tnow+told;	    
				told=tnow;
				if(tcnt>=ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
			}  
		};										    
	}  
	//��ʱnms
	//nms:Ҫ��ʱ��ms��
	//nms:0~65535
	void delay_ms(uint32_t nms)
	{	
		if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//ϵͳ�Ѿ�����
		{		
			if(nms>=fac_ms)						//��ʱ��ʱ�����OS������ʱ������ 
			{ 
				vTaskDelay(nms/fac_ms);	 		//FreeRTOS��ʱ
			}
			nms%=fac_ms;						//OS�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ    
		}
		delay_us((uint32_t)(nms*1000));				//��ͨ��ʽ��ʱ
	}

	//��ʱnms,���������������
	//nms:Ҫ��ʱ��ms��
	void delay_xms(uint32_t nms)
	{
		uint32_t i;
		for(i=0;i<nms;i++) delay_us(1000);
	}
#else  //����ucosʱ
	//��ʱnus
	//nusΪҪ��ʱ��us��.	
	//ע��:nus��ֵ,��Ҫ����798915us(���ֵ��2^24/fac_us@fac_us=21)
	void delay_us(uint32_t nus)
	{		
		uint32_t temp;	    	 
		SysTick->LOAD=nus*fac_us; 				//ʱ�����	  		 
		SysTick->VAL=0x00;        				//��ռ�����
		SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //��ʼ���� 	 
		do
		{
			temp=SysTick->CTRL;
		}while((temp&0x01)&&!(temp&(1<<16)));	//�ȴ�ʱ�䵽��   
		SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//�رռ�����
		SysTick->VAL =0X00;       				//��ռ����� 
	}
	//��ʱnxms,���������������
	//nms:Ҫ��ʱ��ms��
	void delay_xms(u32 nms)
	{
		u32 i;
		for(i=0;i<nms;i++) delay_us(1000);
	}	
	//��ʱnms 
	//nms:0~65535
	void delay_ms(uint32_t nms)
	{	 	 
		uint8_t repeat=nms/540;						//������540,�ǿ��ǵ�ĳЩ�ͻ����ܳ�Ƶʹ��,
													//���糬Ƶ��248M��ʱ��,delay_xms���ֻ����ʱ541ms������
		uint16_t remain=nms%540;
		while(repeat)
		{
			delay_xms(540);
			repeat--;
		}
		if(remain)delay_xms(remain);
	} 
#endif
/*---------------------------------------------------------------------------*/
#if (configSUPPORT_STATIC_ALLOCATION == 1)
	#ifndef   __WEAK
	  #define __WEAK                                 __attribute__((weak))
	#endif
	/* External Idle and Timer task static memory allocation functions */
	extern void vApplicationGetIdleTaskMemory  (StaticTask_t **ppxIdleTaskTCBBuffer,  StackType_t **ppxIdleTaskStackBuffer,  uint32_t *pulIdleTaskStackSize);
	extern void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

	/*
	  vApplicationGetIdleTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
	  equals to 1 and is required for static memory allocation support.
	*/
	__WEAK void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	  /* Idle task control block and stack */
	  static StaticTask_t Idle_TCB;
	  static StackType_t  Idle_Stack[configMINIMAL_STACK_SIZE];

	  *ppxIdleTaskTCBBuffer   = &Idle_TCB;
	  *ppxIdleTaskStackBuffer = &Idle_Stack[0];
	  *pulIdleTaskStackSize   = (uint32_t)configMINIMAL_STACK_SIZE;
	}

	/*
	  vApplicationGetTimerTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
	  equals to 1 and is required for static memory allocation support.
	*/
	__WEAK void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
	  /* Timer task control block and stack */
	  static StaticTask_t Timer_TCB;
	  static StackType_t  Timer_Stack[configTIMER_TASK_STACK_DEPTH];

	  *ppxTimerTaskTCBBuffer   = &Timer_TCB;
	  *ppxTimerTaskStackBuffer = &Timer_Stack[0];
	  *pulTimerTaskStackSize   = (uint32_t)configTIMER_TASK_STACK_DEPTH;
	}
#endif
#ifdef __cplusplus
}
#endif			 


