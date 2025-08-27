
#include "uart3Link_Task.h"
#include "usart3.h"
#include "osQueueSemap.h"
#include "Modbus.h"
xQueueHandle uarts3RecvQueue =NULL;
xQueueHandle uarts3SendQueue =NULL;
TimerHandle_t xAutoReloadTimer;
BaseType_t xTimer1Started;
SonarDate uartData_t;
uint8_t modDevOnline = 1;
uint8_t modDev1Offilne = 1;
uint8_t modDev2Offilne = 1;
static xSemaphoreHandle uart3CompleteBinary = NULL;
static uint8_t uart3Interfa_CallBack(const void *payload,uint16_t *payload_len);
static void sendReadSensorValue_Cmd(uint8_t Dev_t);

static void checkDevTimerCallback( TimerHandle_t xTimer )
{ 
	switch(modDevOnline){
		case 0x01:{							//发送读取第一个超声波数据
			sendReadSensorValue_Cmd(1);
			if(0 == modDev2Offilne){
				modDevOnline = 0x02;		//超声波2没有离线则读取超声波2数据
			}
		}break;
		case 0x02:{							//发送读取第二个超声波数据
			sendReadSensorValue_Cmd(2);
			if(0 == modDev1Offilne){
				modDevOnline = 0x01;		//超声波1没有离线则读取超声波2数据
			}
		}break;
	}
}
static void uart3SendRxQueueTimeout(stcATBuff *buff_t)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uarts3RecvQueue, buff_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static bool uart3ReadRxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(uarts3RecvQueue,buff_t, 1000) == pdTRUE)
	{
		return true;
	}
	memset(buff_t->DataBuff,0,sizeof(buff_t->DataBuff));
	buff_t->length_t = 0;
	return false;
}
uint8_t uart3SendTxQueueTimeout(void *buff_t)
{
	stcATBuff *pBuff_t = (stcATBuff *)buff_t;
	if (xQueueSend(uarts3SendQueue,pBuff_t,1000) == pdTRUE)
	{
		return true;
	}
	return false;
}
static void sendReadSensorValue_Cmd(uint8_t Dev_t){
	switch(Dev_t){
		case 0:{										//先自动读取一次超声波数据(超声波1,超声波2)
			if(configParam.sonarCfg.En1== 2){
				MB_ReadHoldingReg_03H(0x01,0x0100,1);
			}else{
				uartData_t.Sonar1 = 0;
			}
			vTaskDelay(500);
			if(configParam.sonarCfg.En2== 2){
				MB_ReadHoldingReg_03H(0x02,0x0100,1);
			}else{
				uartData_t.Sonar2 = 0;
			}
		}break;
		case 1:{										//读取超声波1数据
			MB_ReadHoldingReg_03H(0x01,0x0100,1);
		}break;
		case 2:{										//读取超声波2数据
			MB_ReadHoldingReg_03H(0x02,0x0100,1);
		}
	}
}
static uint8_t readSensor_Processing(uint8_t *Src_t,uint16_t len_t){
	uint16_t crcVal = 0;
	if(len_t<4){
		return 1;
	}
	switch((uint8_t)(Src_t[0])){
		case 1:{																	//解析超声波1数据
			crcVal = Src_t[len_t - 1]<<8 | Src_t[len_t - 2];
			if(crcVal == MB_CRC16(Src_t,len_t-2)){
				uartData_t.Sonar1 = (uint16_t)(Src_t[len_t - 4]<<8|Src_t[len_t -3]);
			}
			modDev1Offilne = 0;														//超声波1是否离线,没有离线
//			sendReadSensorValue_Cmd(1);
			return 0;
		}break;
		case 2:{																	//解析超声波2数据
			crcVal = Src_t[len_t - 1]<<8 | Src_t[len_t - 2];
			if(crcVal == MB_CRC16(Src_t,len_t-2)){
				uartData_t.Sonar2 = (uint16_t)(Src_t[len_t - 4]<<8|Src_t[len_t -3]);
			}
			modDev2Offilne = 0;														//超声波2是否离线,没有离线
//			sendReadSensorValue_Cmd(2);
			return 0;
		}break;
	}
	return 1;
}
SonarDate getUartSonarValue(void){	
	return uartData_t;
}
static bool uart3ReadTxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(uarts3SendQueue,buff_t, portMAX_DELAY) == pdTRUE)	/*接收usbDataDelivery(1024个容量)消息*/
	{
		xSemaphoreTake(uart3CompleteBinary, 0);
		taskENTER_CRITICAL();
		u3DataFrame_Send(buff_t->DataBuff,buff_t->length_t);
		taskEXIT_CRITICAL();
			/* Wait the DMA send over & Start next */
		xSemaphoreTake(uart3CompleteBinary, 1000);
		return true;
	}
	return false;
}
static void uart3DmaSendOver_callback(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart3CompleteBinary, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void uart3Rx_HTask(void *pvParameters)
{	
	stcATBuff uart3RxBuff_t;
    for( ;; ){	
		if(uart3ReadRxQueueTimeout(&uart3RxBuff_t)){
			switch(configParam.IsUsbLink){
				#ifndef Custom
				case RELAID_USBtoUR3:		//USB连接上位机RS485(UART3)转发,         		把RS485(UART3)接收的数据通过USB带协议转发出去
				case RELAID_UR2toUR3:		//BLE(UART2)连接上位机RS485(UART3)转发,  		把RS485(UART3)接收的数据通过BLE(UART2)带协议转发出去
				case RELAID_UR5toUR3:		//RS232(UART5)连接上位机RS485(UART3)转发,		把RS485(UART3)接收的数据通过RS232(UART5)带协议转发出去
				case RELAID_UR1toUR3:{		//UART1连接上位机RS485(UART3)转发,       		把RS485(UART3)接收的数据通过UART1带协议转发出去
					memmove(&uart3RxBuff_t.DataBuff[PACK_DATA_INDEX],uart3RxBuff_t.DataBuff,uart3RxBuff_t.length_t);
					uart3RxBuff_t.DataBuff[PACK_LEN_H_INDEX] = uart3RxBuff_t.length_t>>8;
					uart3RxBuff_t.DataBuff[PACK_LEN_L_INDEX] = uart3RxBuff_t.length_t;
					uart3RxBuff_t.DataBuff[RESULTS_TYPE_INDEX] = RELAID_MODBUS;
					uart3RxBuff_t.length_t = sendProcessing(uart3RxBuff_t.DataBuff,CMD_PORT_RELAID);
					communicationSend_Struct(&uart3RxBuff_t,configParam.IsUsbLink);
				}break;
				case RELAID_UR1autoUR3:		//UART1连接上位机RS485(UART3)透明转发,       	把RS485(UART3)接收到的数据通过UART1直接转发出去
				case RELAID_UR5autoUR3:		//RS232(UART5)连接上位机RS485(UART3)透明转发,	把RS485(UART3)接收到的数据通过UART5直接转发出去
				case RELAID_UR2autoUR3:		//UART2连接上位机RS485(UART3)透明转发,       	把RS485(UART3)接收到的数据通过UART2直接转发出去
				case RELAID_USBautoUR3:{	//USB连接上位机RS485(UART3)透明转发,         	把RS485(UART3)接收到的数据通过USB直接转发出去
					communicationSend_Struct(&uart3RxBuff_t,configParam.IsUsbLink);
				}
				#endif
				default:{
					if(readSensor_Processing(uart3RxBuff_t.DataBuff,uart3RxBuff_t.length_t)){
						uart3SendTxQueueTimeout(&uart3RxBuff_t);
					}
				}break;
			}
			
		}
	}
}
static void uart3Tx_HTask(void *pvParameters)
{	
	stcATBuff uart3TxBuff_t;
	sendReadSensorValue_Cmd(0);
    for( ;; ) {	
		uart3ReadTxQueueTimeout(&uart3TxBuff_t);
	}
}
void uart3Link_TaskInit()
{	
	if(configParam.ROS_SERIALx != SERIAL3){
		uarts3RecvQueue = xQueueCreate(5, sizeof(stcATBuff));
		uarts3SendQueue = xQueueCreate(5, sizeof(stcATBuff));
		uart3CompleteBinary = xSemaphoreCreateBinary();
		xSemaphoreTake(uart3CompleteBinary, 0);
		u3SetDmaSendOver_func(uart3DmaSendOver_callback);
		u3SetDmaRead_func(uart3SendRxQueueTimeout);
		registeMBSend(uart3SendTxQueueTimeout);
		USART3_Init(115200);
		#ifndef Custom
		registerComSendCallback(uart3SendTxQueueTimeout,RELAID_MODBUS);
		#endif
		if(configParam.sonarCfg.En1 == 2 || configParam.sonarCfg.En2 == 2 || \
		   configParam.sonarCfg.En3 == 2 || configParam.sonarCfg.En4 == 2){
			xAutoReloadTimer = xTimerCreate("AutoReload",150,pdTRUE,0,checkDevTimerCallback );
			xTimer1Started = xTimerStart( xAutoReloadTimer, 0 );
		}
		xTaskCreate(uart3Rx_HTask,(const char *)"uart3Rx_HTask",256, NULL,uart3Rx_Pri, NULL);
		xTaskCreate(uart3Tx_HTask,(const char *)"uart3Tx_HTask",256, NULL,uart3Tx_Pri, NULL);
	}
}