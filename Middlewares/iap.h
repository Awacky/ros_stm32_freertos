#ifndef __IAP_H
#define __IAP_H

#include "hw_config.h"

typedef  void (*iapfun)(void);					//����һ���������͵Ĳ���.     
void iap_load_app(uint32_t appxaddr);			//��ת��APP����ִ��

#endif







































