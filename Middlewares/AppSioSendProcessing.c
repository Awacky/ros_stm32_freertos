#include "hw_config.h"
#include "hcBle_Task.h"
#include "uart5Link_Task.h"
#include "usbLink_Task.h"
#include "uart1Link_Task.h"
#ifdef __cplusplus
extern "C" {
#endif
void AppSioSendProcessing(stcATBuff *buff_t){
	if((1 == configParam.IsUsbLink) || (11 == configParam.IsUsbLink) || 
	   (21 == configParam.IsUsbLink)|| (31 == configParam.IsUsbLink)){
		switch(configParam.IsUsbLink){
			case 1:
				usbSendTxQueueTimeout(buff_t);
				break;
			case 11:
				uart2SendTxQueueTimeout(buff_t);
				break;
			case 21:
				uart5SendTxQueueTimeout(buff_t);
				break;
			case 31:
				uart1SendTxQueueTimeout(buff_t);
				break;
		}
	}
}
void AppSioSendProcessingProt(stcATBuff *buff_t,uint8_t SioName){
	switch(SioName){
		case USB_SIO:
			usbSendTxQueueTimeout(buff_t);
			break;
		case SERIAL2:
			uart2SendTxQueueTimeout(buff_t);
			break;
		case SERIAL5:
			uart5SendTxQueueTimeout(buff_t);
			break;
		case SERIAL1:
			uart1SendTxQueueTimeout(buff_t);
			break;
	}
}
#ifdef __cplusplus
}
#endif



