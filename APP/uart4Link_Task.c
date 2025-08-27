
#include "uart4Link_Task.h"
#include "uart4.h"
#include "osQueueSemap.h"
#include "sbus.h"
#include "OledShow_Task.h"
#include "crc_calc.h"
#include "moveBase_Task.h"

static sbusCfg getValue_t;

gps_report getGpsVal_t;
static stcMoveVel strMV_sbus;
static uint16_t velRatioZ = 3;
xQueueHandle uart4RecvQueue =NULL;
xQueueHandle uart4SendQueue =NULL;
static xSemaphoreHandle uart4CompleteBinary = NULL;
static uint8_t uart4Interfa_CallBack(const void *payload,uint16_t *payload_len);
static void uart4SendRxQueueTimeout(stcATBuff *buff_t)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uart4RecvQueue, buff_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static bool uart4ReadRxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(uart4RecvQueue,buff_t, portMAX_DELAY) == pdTRUE){
		return true;
	}
	memset(buff_t->DataBuff,0,sizeof(buff_t->DataBuff));
	buff_t->length_t = 0;
	return false;
}
uint8_t uart4SendTxQueueTimeout(void *buff_t)
{
	stcATBuff *pBuff_t = (stcATBuff *)buff_t;
	if (xQueueSend(uart4SendQueue,pBuff_t,1000) == pdTRUE){
		return true;
	}
	return false;
}
static bool uart4ReadTxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(uart4SendQueue,buff_t, 1000) == pdTRUE)	/*接收usbDataDelivery(1024个容量)消息*/
	{
		xSemaphoreTake(uart4CompleteBinary, 0);
		taskENTER_CRITICAL();
		u4DataFrame_Send(buff_t->DataBuff,buff_t->length_t);
		taskEXIT_CRITICAL();
			/* Wait the DMA send over & Start next */
		xSemaphoreTake(uart4CompleteBinary, 1000);
		return true;
	}
	return false;
}
static void uart4DmaSendOver_callback(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart4CompleteBinary, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void uart4Rx_HTask(void *pvParameters)
{	
	stcATBuff uart4RxBuff_t;
	uint8_t gDataIndex=6;
	uint16_t ubxCrcVal = 0;
	int32_t gGpsVal = 0;
    for( ;; ) 
	{	
		if(uart4ReadRxQueueTimeout(&uart4RxBuff_t)){
			
			switch(configParam.IsUsbLink){
				#ifndef Custom
				case RELAID_USBtoUR4:	//USB连接上位机UART4转发,			把RS232(UART5)接收的数据通过USB带协议转发出去
				case RELAID_UR2toUR4:	//BLE(UART2)连接上位机UART4转发,    把RS232(UART5)接收的数据通过BLE(UART2)带协议转发出去
				case RELAID_UR5toUR4:	//RS232(UART5)连接上位机UART4转发,	把RS232(UART5)接收的数据通过RS232(UART5)带协议转发出去
				case RELAID_UR1toUR4:{	//UART1连接上位机UART4转发,			把RS232(UART5)接收的数据通过UART1带协议转发出去
					memmove(&uart4RxBuff_t.DataBuff[PACK_DATA_INDEX],uart4RxBuff_t.DataBuff,uart4RxBuff_t.length_t);
					uart4RxBuff_t.DataBuff[PACK_LEN_H_INDEX] = uart4RxBuff_t.length_t>>8;
					uart4RxBuff_t.DataBuff[PACK_LEN_L_INDEX] = uart4RxBuff_t.length_t;
					uart4RxBuff_t.DataBuff[RESULTS_TYPE_INDEX] = RELAID_UART4;
					uart4RxBuff_t.length_t = sendProcessing(uart4RxBuff_t.DataBuff,CMD_PORT_RELAID);
					communicationSend_Struct(&uart4RxBuff_t,configParam.IsUsbLink);
				}break;
				#endif
				default:{
					if(sbus_decode(uart4RxBuff_t.DataBuff,&getValue_t)){
						if(getValue_t.values[5]>1500 && (getValue_t.failsafe==0)){			//开启手柄控制则解除急停,启用遥控赋值
							strMV_sbus.isLock = 5;
//							velRatioZ = (getValue_t.values[6] - 960) / 100;
							strMV_sbus.VelX = (1500 - getValue_t.values[1])/450.f;			//X
							strMV_sbus.VelZ = (1500 - getValue_t.values[0])/450.f*velRatioZ;//Z
//							strMV_sbus.VelY = (1500 - getValue_t.values[3])/450.f;			//Y
							strMV_sbus.VelY = 0;											//Y
							if(fabs(strMV_sbus.VelX)<0.1)strMV_sbus.VelX = 0;
							if(fabs(strMV_sbus.VelZ)<0.1)strMV_sbus.VelZ = 0;
							if(fabs(strMV_sbus.VelY)<0.1)strMV_sbus.VelY = 0;
							setMoveVelData(&strMV_sbus);
						}else{							//关闭遥控控制,则发送一次停止信号,交由其他信号控制
							if(5 == strMV_sbus.isLock){	//strMV_tp.isLock:0解除急停,1触发急停,2前进减速,3后退急停速,4后退减速
								strMV_sbus.isLock = 0;
								strMV_sbus.VelX = 0;
								strMV_sbus.VelZ = 0;
								strMV_sbus.VelY = 0;
								setMoveVelData(&strMV_sbus);
								strMV_sbus.isLock = 6;
							}
						}
					}else if(uart4RxBuff_t.DataBuff[0] == 0xB5 && uart4RxBuff_t.DataBuff[1] == 0x62 &&
					   uart4RxBuff_t.DataBuff[2] == 0x01 && uart4RxBuff_t.DataBuff[3] == 0x07){
						ubxCrcVal = calc_ubx_checksum(&uart4RxBuff_t.DataBuff[2],uart4RxBuff_t.length_t-4);
						if(ubxCrcVal == (uint16_t)(uart4RxBuff_t.DataBuff[uart4RxBuff_t.length_t-2]|uart4RxBuff_t.DataBuff[uart4RxBuff_t.length_t-1]<<8)){
							ubxCrcVal = uart4RxBuff_t.DataBuff[82]|uart4RxBuff_t.DataBuff[83]<<8;
							if(uart4RxBuff_t.DataBuff[26]>=3 && ubxCrcVal<300){//3颗星并且水平精度因子小于3
								gDataIndex=6;
								getGpsVal_t.tim_pos = uart4RxBuff_t.DataBuff[gDataIndex++];			//0	位置时间戳			
								getGpsVal_t.tim_pos|= uart4RxBuff_t.DataBuff[gDataIndex++]<<8;		//1 位置时间戳	
								getGpsVal_t.tim_pos|= uart4RxBuff_t.DataBuff[gDataIndex++]<<16;		//2 位置时间戳	
								getGpsVal_t.tim_pos|= uart4RxBuff_t.DataBuff[gDataIndex++]<<24;		//3 位置时间戳	
								getGpsVal_t.year  	= uart4RxBuff_t.DataBuff[gDataIndex++];			//4 年
								getGpsVal_t.year   |= uart4RxBuff_t.DataBuff[gDataIndex++]<<8;		//5 年
								getGpsVal_t.month 	= uart4RxBuff_t.DataBuff[gDataIndex++];			//6	月
								getGpsVal_t.day   	= uart4RxBuff_t.DataBuff[gDataIndex++];			//7	日
								getGpsVal_t.hour  	= uart4RxBuff_t.DataBuff[gDataIndex++];			//8	时
								getGpsVal_t.min   	= uart4RxBuff_t.DataBuff[gDataIndex++];			//9	分
								getGpsVal_t.sec   	= uart4RxBuff_t.DataBuff[gDataIndex++];			//10秒
								gDataIndex = 30;													//24
								gGpsVal  = uart4RxBuff_t.DataBuff[gDataIndex];gDataIndex++;
								gGpsVal |= uart4RxBuff_t.DataBuff[gDataIndex]<<8;gDataIndex++; 
								gGpsVal |= uart4RxBuff_t.DataBuff[gDataIndex]<<16;gDataIndex++;	
								gGpsVal |= uart4RxBuff_t.DataBuff[gDataIndex]<<24;gDataIndex++;
								getGpsVal_t.lon = gGpsVal;											//纬度	
																									//28
								gGpsVal  = uart4RxBuff_t.DataBuff[gDataIndex];gDataIndex++;
								gGpsVal |= uart4RxBuff_t.DataBuff[gDataIndex]<<8;gDataIndex++; 
								gGpsVal |= uart4RxBuff_t.DataBuff[gDataIndex]<<16;gDataIndex++;	
								gGpsVal |= uart4RxBuff_t.DataBuff[gDataIndex]<<24;gDataIndex++;
								getGpsVal_t.lat = gGpsVal;											//经度
								gDataIndex = 42;													//36
								gGpsVal  = uart4RxBuff_t.DataBuff[gDataIndex];gDataIndex++;
								gGpsVal |= uart4RxBuff_t.DataBuff[gDataIndex]<<8;gDataIndex++; 
								gGpsVal |= uart4RxBuff_t.DataBuff[gDataIndex]<<16;gDataIndex++;	
								gGpsVal |= uart4RxBuff_t.DataBuff[gDataIndex]<<24;gDataIndex++;
								getGpsVal_t.alt = gGpsVal;											//海拔高度
							}
							oledUpdataGpsVal(&getGpsVal_t);
						}
					} else{
					uart4SendTxQueueTimeout(&uart4RxBuff_t);
					}
				}break;
			}
		}
	}
}
static void uart4Tx_HTask(void *pvParameters)
{	
	stcATBuff uart4TxBuff_t;
    for( ;; ) {	
		uart4ReadTxQueueTimeout(&uart4TxBuff_t);
	}
}
void uart4Link_TaskInit()
{	
	if(configParam.ROS_SERIALx != SERIAL4){
		getValue_t.max_values = 10;							//目前使用的是10通道遥控器
		uart4RecvQueue = xQueueCreate(5, sizeof(stcATBuff));
		uart4SendQueue = xQueueCreate(5, sizeof(stcATBuff));
		uart4CompleteBinary = xSemaphoreCreateBinary();
		xSemaphoreTake(uart4CompleteBinary, 0);
		u4SetDmaSendOver_func(uart4DmaSendOver_callback);
		u4SetDmaRead_func(uart4SendRxQueueTimeout);
		#ifndef Custom
		registerComSendCallback(uart4SendTxQueueTimeout,RELAID_UART4);
		#endif
		UART4_Init(9600);
		xTaskCreate(uart4Rx_HTask,(const char *)"uart4Rx_HTask",128, NULL,uart4Rx_Pri, NULL);
		xTaskCreate(uart4Tx_HTask,(const char *)"uart4Tx_HTask",128, NULL,uart4Tx_Pri, NULL);
	}
}
//B5 62 06 01 03 00 01 07 00 12 50 禁用PVT
//B5 62 06 01 03 00 01 07 01 13 51 启用PVT
//B5 62 0A 04 00 00 0E 34
//B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 17 31 BF //保存当前配置

//B5 62 06 01 03 00 F0 0A 00 04 23 禁用NMEA
//B5 62 06 01 03 00 F0 09 00 03 21 禁用NMEA

//B5 62 06 01 03 00 F0 0A 01 05 24 启用NMEA
//B5 62 06 01 03 00 F0 09 01 04 22 启用NMEA

