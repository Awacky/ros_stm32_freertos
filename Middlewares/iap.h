#ifndef __IAP_H
#define __IAP_H

#include "hw_config.h"

typedef  void (*iapfun)(void);					//定义一个函数类型的参数.     
void iap_load_app(uint32_t appxaddr);			//跳转到APP程序执行

#endif







































