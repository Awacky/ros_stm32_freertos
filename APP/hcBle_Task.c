
#include "hcBle_Task.h"
#include "usart2.h"
#include "osQueueSemap.h"

#include "config_param.h"
#include "SEGGER_RTT.h"
//#include "CustomPlot.h"

#include "moveBase_Task.h"
xQueueHandle uart2RecvQueue =NULL;
xQueueHandle uart2SendQueue =NULL;
static xSemaphoreHandle uart2CompleteBinary = NULL;
static uint8_t uart2Interfa_CallBack(const void *payload,uint16_t *payload_len);
static void uart2SendRxQueueTimeout(stcATBuff *buff_t)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uart2RecvQueue, buff_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static bool uart2ReadRxQueueTimeout(stcATBuff *buff_t)
{
	if(xQueueReceive(uart2RecvQueue,buff_t, 5) == pdTRUE)
	{
		return true;
	}
	memset(buff_t->DataBuff,0,sizeof(buff_t->DataBuff));
	buff_t->length_t = 0;
	return false;
}
uint8_t uart2SendTxQueueTimeout(void *buff_t)
{
	stcATBuff *pBuff_t = (stcATBuff *)buff_t;
	if (xQueueSend(uart2SendQueue,pBuff_t,1000) == pdTRUE){
		return true;
	}
	return false;
}
static bool uart2ReadTxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(uart2SendQueue,buff_t,1000) == pdTRUE)
	{	
		xSemaphoreTake(uart2CompleteBinary, 0);
		taskENTER_CRITICAL();
		u2DataFrame_Send(buff_t->DataBuff,buff_t->length_t);
		taskEXIT_CRITICAL();
			/* Wait the DMA send over & Start next */
		xSemaphoreTake(uart2CompleteBinary, 1000);
		return true;
	}
	return false;
}
static void uart2DmaSendOver_callback(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart2CompleteBinary, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void hcBleRx_HTask(void *pvParameters)
{	
	stcATBuff uart2Buff_t;
	uint8_t gManage_t = 0;
	float bleSetSpeed = 0.3;
	stcTwist bleTwist_p;
	while(1){
		if(uart2ReadRxQueueTimeout(&uart2Buff_t)){
			if(uart2Buff_t.length_t==1){
				switch(uart2Buff_t.DataBuff[0]){
					case 0x41:{									//前进
						bleTwist_p.linearVelX  = bleSetSpeed;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = 0.0;
					}break;
					case 0x42:{									//右前转
						bleTwist_p.linearVelX  = bleSetSpeed;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = -bleSetSpeed;
					}break;
					case 0x43:{									//右自转
						bleTwist_p.linearVelX  = 0.0;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = -bleSetSpeed;
					}break;
					case 0x44:{									//右后转
						bleTwist_p.linearVelX  = -bleSetSpeed;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = -bleSetSpeed;
					}break;
					case 0x45:{									//后退
						bleTwist_p.linearVelX  = -bleSetSpeed;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = 0.0;
					}break;
					case 0x46:{									//左后转
						bleTwist_p.linearVelX  = -bleSetSpeed;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = bleSetSpeed;
					}break;	
					case 0x47:{									//左自转
						bleTwist_p.linearVelX  = 0.0;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = bleSetSpeed;
					}break;
					case 0x48:{									//左前转
						bleTwist_p.linearVelX  = bleSetSpeed;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = bleSetSpeed;
					}break;
					case 0x58:{
						bleSetSpeed += 0.2;
						if(bleSetSpeed>3)bleSetSpeed = 3;
					}break;
					case 0x59:{
						bleSetSpeed -= 0.2;
						if(bleSetSpeed<0.2)bleSetSpeed = 0.2;
					}break;
					case 0x5A:{
						bleTwist_p.linearVelX  = 0.0;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = 0.0;
						configParam.ControlMode = 1;
					}break;
					default:{
						bleTwist_p.linearVelX  = 0.0;
						bleTwist_p.linearVelY  = 0.0;
						bleTwist_p.angularVelZ = 0.0;
					}break;
				}
				twistlinkSendPacket(&bleTwist_p);
				gManage_t = 0xFE;
			}else{
				#ifndef Custom
				gManage_t=replyProcessing(uart2Buff_t.DataBuff,(uint8_t)uart2Buff_t.DataBuff[PACK_CMD_INDEX],(uint16_t *)&(uart2Buff_t.length_t),1);
				switch(gManage_t){
					case 1:{							//参数保存
						configParamGiveSemaphore();
					}break;
					case 2:{							//恢复默认参数
						restoreDefaultParam();
						configParamGiveSemaphore();
					}break;
					case 3:{							//使能UART2主动上报
						configParam.IsUsbLink = UR2_Enabled;		
					}break;
					case 4:{							//禁用UART2主动上报
						configParam.IsUsbLink = UR2_Disable;		
					}break;
					case 5:{							//串口2连接上位机CAN数据转发
						configParam.IsUsbLink = RELAID_UR2toCAN;	
					}break;
					case 6:{							//串口2连接上位机RS485(UART3)数据转发
						configParam.IsUsbLink = RELAID_UR2toUR3;	
					}break;
					case 7:{							//串口2连接上位机RS232(UART5)数据转发
						configParam.IsUsbLink = RELAID_UR2toUR5;	
					}break;
					case 9:{							//串口2连接上位机UART4数据转发
						configParam.IsUsbLink = RELAID_UR2toUR4;	
					}break;
					case 10:{							//串口2连接上位机UART1数据转发
						configParam.IsUsbLink = RELAID_UR2toUR1;	
					}break;
					case 11:{							//RS485(UART3)透明转发
						configParam.IsUsbLink = RELAID_UR2autoUR3;
					}break;
					case 13:{							//RS232(UART5)透明转发
						configParam.IsUsbLink = RELAID_UR2autoUR5;
					}break;
					case UP_CMD_COPY_APP:{				//重启在Boot拷贝升级固件到App中并重启
						uart2SendTxQueueTimeout(&uart2Buff_t);
						vTaskDelay(50);
						SoftReset_fun();					
					}break;
					case UP_CMD_COPY_BOOT:{				//拷贝升级固件到Boot中并重启
						uart2SendTxQueueTimeout(&uart2Buff_t);
						vTaskDelay(50);
						uart2Buff_t.DataBuff[PACKET_CMD_INDEX] = UP_BOOT_GOTO_APP;
						if(appFirCopyBoot()){	   					//拷贝成功并重启，拷贝失败则回复上位机
							uart2Buff_t.DataBuff[PACKET_RESULT_INDEX]  = PACKET_ACK_OK;	
						}else{
							uart2Buff_t.DataBuff[PACKET_RESULT_INDEX]  = PACKET_ACK_GoToApp_ERROR;	
						}				
						uart2SendTxQueueTimeout(&uart2Buff_t);
					}break;
					default:{
						switch(configParam.IsUsbLink){
							case RELAID_USBtoUR2:		//USB连接上位机BLE(UART2)转发,把BLE(UART2)接收的数据通过USB带协议转发出去
							case RELAID_UR5toUR2:		//RS232(UART5)连接上位机BLE(UART2)转发,把BLE(UART2)接收的数据通过RS232(UART5)带协议转发出去
							case RELAID_UR1toUR2:{		//UART1连接上位机BLE(UART2)转发,把BLE(UART2)接收的数据通过UART1带协议转发出去
								memmove(&uart2Buff_t.DataBuff[PACK_DATA_INDEX],uart2Buff_t.DataBuff,uart2Buff_t.length_t);
								uart2Buff_t.DataBuff[PACK_LEN_H_INDEX] = uart2Buff_t.length_t>>8;
								uart2Buff_t.DataBuff[PACK_LEN_L_INDEX] = uart2Buff_t.length_t;
								uart2Buff_t.DataBuff[RESULTS_TYPE_INDEX] = RELAID_BLE;
								uart2Buff_t.length_t = sendProcessing(uart2Buff_t.DataBuff,CMD_PORT_RELAID);
								communicationSend_Struct(&uart2Buff_t,configParam.IsUsbLink);	
								gManage_t = 0xFE;
							}break;
							case RELAID_UR2autoUR3:{	//UART2连接上位机RS485(UART3)透明转发,把BLE(UART2)接收的数据通过RS485(UART3)直接转发出去
								communicationSend_Struct(&uart2Buff_t,RELAID_UR3toUR3);	
								gManage_t = 0xFE;
							}break;
							case RELAID_UR2autoUR5:{	//UART2连接上位机RS232(UART5)透明转发,把BLE(UART2)2接收的数据通过RS232(UART5)直接转发出去
								communicationSend_Struct(&uart2Buff_t,UR5_Enabled);	
								gManage_t = 0xFE;
							}break;
							case RELAID_UR1autoUR2:		//UART1连接上位机BLE(UART2)透明转发,把BLE(UART2)接收到的数据通过UART1直接转发出去
							case RELAID_USBautoUR2:{	//USB连接上位机BLE(UART2)透明转发,把BLE(UART2)接收到的数据通过USB直接转发出去
								communicationSend_Struct(&uart2Buff_t,configParam.IsUsbLink);	
								gManage_t = 0xFE;
							}break;
							default:{
								
							}break;
						}
					}break;
				}
				#endif
			}
			if(0xFE!=gManage_t){						//不用回复
				uart2SendTxQueueTimeout(&uart2Buff_t);
			}			
		}
	}
}
static void hcBleTx_HTask(void *pvParameters)
{	
	stcATBuff uart2TxBuff_t;
    for( ;; ) {	
		uart2ReadTxQueueTimeout(&uart2TxBuff_t);
	}
}
void hcBle_TaskInit()
{	
	if(configParam.ROS_SERIALx != SERIAL2){
		uart2RecvQueue = xQueueCreate(5, sizeof(stcATBuff));
		uart2SendQueue = xQueueCreate(5, sizeof(stcATBuff));
		uart2CompleteBinary = xSemaphoreCreateBinary();
		xSemaphoreTake(uart2CompleteBinary, 0);
		u2SetDmaSendOver_func(uart2DmaSendOver_callback);
		u2SetDmaRead_func(uart2SendRxQueueTimeout);
		#ifndef Custom
		registerComSendCallback(uart2SendTxQueueTimeout,RELAID_BLE);
		#endif
		USART2_Init(9600);
		xTaskCreate(hcBleRx_HTask,(const char *)"hcBleRx_HTask",256, NULL,hcBleRx_Pri, NULL);
		xTaskCreate(hcBleTx_HTask,(const char *)"hcBleTx_HTask",256, NULL,hcBleTx_Pri, NULL);
	}
}