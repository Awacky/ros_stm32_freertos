#include "device_storage.h"
#include "iap.h" 
iapfun jump2app; 
//��ת��Ӧ�ó����
//appxaddr:�û�������ʼ��ַ.
void iap_load_app(uint32_t appxaddr)
{
	if((*(uint32_t*)APP_ADDR)==0xFFFFFFFF)return;
	#ifdef STM32F40_41xxx
		if(((*(uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)	//���ջ����ַ�Ƿ�Ϸ�.
	#elif AT32F40x	
		if(((*(uint32_t*)appxaddr) - 0x20000000) < (224 * 1024))
	#endif	
	{ 
		jump2app=(iapfun)*(uint32_t*)(appxaddr+4);		//�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)
		__set_MSP(*(uint32_t*)appxaddr);           		//��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
		for(int i = 0; i < 8; i++){			
			NVIC->ICER[i] = 0xFFFFFFFF;					/* �ر��ж�*/
			NVIC->ICPR[i] = 0xFFFFFFFF;					/* ����жϱ�־λ */
		}
		jump2app();										//��ת��APP.
	}
}		 














