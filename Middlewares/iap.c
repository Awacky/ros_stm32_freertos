#include "device_storage.h"
#include "iap.h" 
iapfun jump2app; 
//跳转到应用程序段
//appxaddr:用户代码起始地址.
void iap_load_app(uint32_t appxaddr)
{
	if((*(uint32_t*)APP_ADDR)==0xFFFFFFFF)return;
	#ifdef STM32F40_41xxx
		if(((*(uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)	//检查栈顶地址是否合法.
	#elif AT32F40x	
		if(((*(uint32_t*)appxaddr) - 0x20000000) < (224 * 1024))
	#endif	
	{ 
		jump2app=(iapfun)*(uint32_t*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)
		__set_MSP(*(uint32_t*)appxaddr);           		//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		for(int i = 0; i < 8; i++){			
			NVIC->ICER[i] = 0xFFFFFFFF;					/* 关闭中断*/
			NVIC->ICPR[i] = 0xFFFFFFFF;					/* 清除中断标志位 */
		}
		jump2app();										//跳转到APP.
	}
}		 














