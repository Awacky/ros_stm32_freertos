#ifdef __cplusplus
extern "C" {
#endif

#include "delay.h"

#if SYSTEM_SUPPORT_OS
	#include "FreeRTOS.h"					//支持OS时，使用	  		  
	#include "task.h"					
#endif

static uint8_t  fac_us=0;							//us延时倍乘数			   
static uint16_t fac_ms=0;							//ms延时倍乘数,在os下,代表每个节拍的ms数
static uint32_t sysTickCnt = 0;
#if SYSTEM_SUPPORT_OS
	extern void xPortSysTickHandler(void);				
#endif

//systick中断服务函数,使用OS时用到
void SysTick_Handler(void)
{	
	#if SYSTEM_SUPPORT_OS
		if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED){//系统已经运行
			xPortSysTickHandler();
		}else{
			sysTickCnt++;
		}
	#else
		sysTickCnt++;
	#endif

}
		   
//初始化延迟函数
//SYSTICK的时钟固定为AHB时钟，基础例程里面SYSTICK时钟频率为AHB/8
//这里为了兼容FreeRTOS，所以将SYSTICK的时钟频率改为AHB的频率！
//SYSCLK:系统时钟频率
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
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);								//设置系统中断优先级分组4 16
		SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); 							//SysTick 时钟不分频
	}
	#elif  defined(AT32F40x)
	{	
		#ifndef BOOTLOADER
		nvic_vector_table_set(NVIC_VECTTAB_FLASH,APP_ADDR - NVIC_VECTTAB_FLASH);
		#endif
//		#ifdef BOOTLOADER
//		nvic_vector_table_set(NVIC_VECTTAB_FLASH,BOOT_ADDR - NVIC_VECTTAB_FLASH);
//		#endif
		nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);							//设置系统中断优先级分组4 16
		system_clock_config();														//时钟配置
		systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);				//SysTick 时钟不分频
		SYSCLK = 240;
	}
	#endif
	fac_us=SYSCLK;								//不论是否使用OS,fac_us都需要使用
	#if SYSTEM_SUPPORT_OS
		reload=SYSCLK;							//每秒钟的计数次数 单位为M	   
		reload*=1000000/configTICK_RATE_HZ;		//根据delay_ostickspersec设定溢出时间
												//reload为24位寄存器,最大值:16777216,在168M下,约合0.0998s左右	
		fac_ms=1000/configTICK_RATE_HZ;			//代表OS可以延时的最少单位	   
		SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//开启SYSTICK中断
		SysTick->LOAD=reload; 					//每1/configTICK_RATE_HZ断一次	
		SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开启SYSTICK   
	#else
		fac_ms=(uint16_t)fac_us*1000;				//非OS下,代表每个ms需要的systick时钟数   
	#endif 	
}								    


#if SYSTEM_SUPPORT_OS 						//如果需要支持OS.
	/********************************************************
	*getSysTickCnt()
	*调度开启之前 返回 sysTickCnt
	*调度开启之前 返回 xTaskGetTickCount()
	*********************************************************/
	uint32_t getSysTickCnt(void)
	{
		if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
			return xTaskGetTickCount();
		else
			return sysTickCnt;
	}	
	//延时nus
	//nus:要延时的us数.	
	//nus:0~204522252(最大值即2^32/fac_us@fac_us=168)	    								   
	void delay_us(uint32_t nus)
	{		
		uint32_t ticks;
		uint32_t told,tnow,tcnt=0;
		uint32_t reload=SysTick->LOAD;				//LOAD的值	    	 
		ticks=nus*fac_us; 						//需要的节拍数 
		told=SysTick->VAL;        				//刚进入时的计数器值
		while(1)
		{
			tnow=SysTick->VAL;	
			if(tnow!=told)
			{	    
				if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
				else tcnt+=reload-tnow+told;	    
				told=tnow;
				if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
			}  
		};										    
	}  
	//延时nms
	//nms:要延时的ms数
	//nms:0~65535
	void delay_ms(uint32_t nms)
	{	
		if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
		{		
			if(nms>=fac_ms)						//延时的时间大于OS的最少时间周期 
			{ 
				vTaskDelay(nms/fac_ms);	 		//FreeRTOS延时
			}
			nms%=fac_ms;						//OS已经无法提供这么小的延时了,采用普通方式延时    
		}
		delay_us((uint32_t)(nms*1000));				//普通方式延时
	}

	//延时nms,不会引起任务调度
	//nms:要延时的ms数
	void delay_xms(uint32_t nms)
	{
		uint32_t i;
		for(i=0;i<nms;i++) delay_us(1000);
	}
#else  //不用ucos时
	//延时nus
	//nus为要延时的us数.	
	//注意:nus的值,不要大于798915us(最大值即2^24/fac_us@fac_us=21)
	void delay_us(uint32_t nus)
	{		
		uint32_t temp;	    	 
		SysTick->LOAD=nus*fac_us; 				//时间加载	  		 
		SysTick->VAL=0x00;        				//清空计数器
		SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //开始倒数 	 
		do
		{
			temp=SysTick->CTRL;
		}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
		SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//关闭计数器
		SysTick->VAL =0X00;       				//清空计数器 
	}
	//延时nxms,不会引起任务调度
	//nms:要延时的ms数
	void delay_xms(u32 nms)
	{
		u32 i;
		for(i=0;i<nms;i++) delay_us(1000);
	}	
	//延时nms 
	//nms:0~65535
	void delay_ms(uint32_t nms)
	{	 	 
		uint8_t repeat=nms/540;						//这里用540,是考虑到某些客户可能超频使用,
													//比如超频到248M的时候,delay_xms最大只能延时541ms左右了
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


