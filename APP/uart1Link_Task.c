
#include "uart1Link_Task.h"
#include "usart1.h"
#include "osQueueSemap.h"
#include "config_param.h"
#include "SEGGER_RTT.h"
#include "CustomPlot.h"
#include "ros_bsp.h"
xQueueHandle uart1RecvQueue =NULL;
xQueueHandle uart1SendQueue =NULL;
static uint8_t u1ToWinTools = 0;
static xSemaphoreHandle uart1CompleteBinary = NULL;
static uint8_t uart1Interfa_CallBack(const void *payload,uint16_t *payload_len);
static void uart1SendRxQueueTimeout(stcATBuff *buff_t)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uart1RecvQueue, buff_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static void uart1ToToolsDisable(void){
	u1ToWinTools = 0;
}
static void uart1ToToolsSendRxQueueTimeout(stcATBuff *buff_t)
{
	u1ToWinTools = 1;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uart1RecvQueue, buff_t, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static bool uart1ReadRxQueueTimeout(stcATBuff *buff_t)
{
	if(xQueueReceive(uart1RecvQueue,buff_t, 5) == pdTRUE)
	{
		return true;
	}
	memset(buff_t->DataBuff,0,sizeof(buff_t->DataBuff));
	buff_t->length_t = 0;
	return false;
}
uint8_t uart1SendTxQueueTimeout(void *buff_t)
{
	stcATBuff *pBuff_t = (stcATBuff *)buff_t;
	if(u1ToWinTools){
		#ifndef Custom
		ros_serial_write_length(pBuff_t->DataBuff,pBuff_t->length_t);
		#endif
		return true;
		
	}
	if (xQueueSend(uart1SendQueue,pBuff_t,1000) == pdTRUE){
		return true;
	}
	return false;
}
static bool uart1ReadTxQueueTimeout(stcATBuff *buff_t)
{
	if (xQueueReceive(uart1SendQueue,buff_t,10) == pdTRUE)
	{	
		xSemaphoreTake(uart1CompleteBinary, 0);
		taskENTER_CRITICAL();
		u1DataFrame_Send(buff_t->DataBuff,buff_t->length_t);
		taskEXIT_CRITICAL();
			/* Wait the DMA send over & Start next */
		xSemaphoreTake(uart1CompleteBinary, 1000);
		return true;
	}
	return false;
}
static void uart1DmaSendOver_callback(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart1CompleteBinary, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
static void uart1Rx_HTask(void *pvParameters)
{	
	stcATBuff uart1Buff_t;
	uint8_t gManage_t = 0;
	while(1){
		if(uart1ReadRxQueueTimeout(&uart1Buff_t)){
			#ifndef Custom
			gManage_t=replyProcessing(uart1Buff_t.DataBuff,(uint8_t)uart1Buff_t.DataBuff[PACK_CMD_INDEX],(uint16_t *)&(uart1Buff_t.length_t),1);
			switch(gManage_t){
				case 1:{							//保存参数
					configParamGiveSemaphore();
				}break;
				case 2:{							//恢复默认参数
					restoreDefaultParam();
					configParamGiveSemaphore();
				}break;
				case 3:{							//使能UART1主动上报
					configParam.IsUsbLink = UR1_Enabled;		
				}break;
				case 4:{							//禁用UART1主动上报
					configParam.IsUsbLink = UR1_Disable;		
				}break;
				case 5:{							//串口1连接上位机CAN数据转发
					configParam.IsUsbLink = RELAID_UR1toCAN;	
				}break;
				case 6:{							//串口1连接上位机RS485(UART3)数据转发
					configParam.IsUsbLink = RELAID_UR1toUR3;	
				}break;
				case 7:{							//串口1连接上位机RS232(UART5)数据转发
					configParam.IsUsbLink = RELAID_UR1toUR5;	
				}break;
				case 8:{							//串口1连接上位机BLE(UART2)数据转发
					configParam.IsUsbLink = RELAID_UR1toUR2;	
				}break;
				case 9:{							//串口1连接上位机UART4数据转发
					configParam.IsUsbLink = RELAID_UR1toUR4;	
				}break;
				case 11:{							//串口1连接上位机RS485(UART3)透明转发
					configParam.IsUsbLink = RELAID_UR1autoUR3;
					gManage_t = 0xFE;
				}break;
				case 12:{							//串口1连接上位机BLE(UART2)透明转发
					configParam.IsUsbLink = RELAID_UR1autoUR2;
					gManage_t = 0xFE;
				}break;
				case UP_CMD_COPY_APP:{		   		//重启在Boot拷贝升级固件到App中并重启
					uart1SendTxQueueTimeout(&uart1Buff_t);
					vTaskDelay(500);
					SoftReset_fun();					
				}break;
				case UP_CMD_COPY_BOOT:{		   		//拷贝升级固件到Boot中并重启
					uart1SendTxQueueTimeout(&uart1Buff_t);
					vTaskDelay(500);					
					if(appFirCopyBoot()){	   					//拷贝成功并重启，拷贝失败则回复上位机
						uart1Buff_t.DataBuff[PACKET_RESULT_INDEX]  = PACKET_ACK_OK;	
					}else{
						uart1Buff_t.DataBuff[PACKET_RESULT_INDEX]  = PACKET_ACK_GoToApp_ERROR;	
					}
					uart1Buff_t.DataBuff[PACKET_CMD_INDEX] = UP_BOOT_GOTO_APP;		
					uart1Buff_t.length_t = sendProcessing(uart1Buff_t.DataBuff,CMD_UPGRADER);
				}break;
				case UP_APP_GO_BOOT:{				//APP跳转到BOOT
					appSetUpFalfg();
					uart1SendTxQueueTimeout(&uart1Buff_t);
					vTaskDelay(500);
					SoftReset_fun();					
				}break;
				default:{
					switch(configParam.IsUsbLink){
						case RELAID_USBtoUR1:		//USB连接上位机UART1转发,把UART1接收的数据通过USB带协议转发出去
						case RELAID_UR5toUR1:		//RS232(UART5)连接上位机UART1转发,把UART1接收的数据通过RS232(UART5)带协议转发出去
						case RELAID_UR2toUR1:{		//BLE(UART2)连接上位机UART1转发,把UART1接收的数据通过BLE(UART2)带协议转发出去
							memmove(&uart1Buff_t.DataBuff[PACK_DATA_INDEX],uart1Buff_t.DataBuff,uart1Buff_t.length_t);
							uart1Buff_t.DataBuff[PACK_LEN_H_INDEX] = uart1Buff_t.length_t>>8;
							uart1Buff_t.DataBuff[PACK_LEN_L_INDEX] = uart1Buff_t.length_t;
							uart1Buff_t.DataBuff[RESULTS_TYPE_INDEX] = RELAID_UART1;
							uart1Buff_t.length_t = sendProcessing(uart1Buff_t.DataBuff,CMD_PORT_RELAID);
							communicationSend_Struct(&uart1Buff_t,configParam.IsUsbLink);
						}break;
						case RELAID_UR1autoUR3:{	//UART1连接上位机RS485(UART3)透明转发,把UART1接收的数据通过RS485(UART3)直接转发出去
							communicationSend_Struct(&uart1Buff_t,RELAID_UR3toUR3);
//							communicationSend_Struct(&uart1Buff_t,RELAID_UR4toUR4);
						}break;
						case RELAID_UR1autoUR2:{	//UART1连接上位机BLE(UART2)透明转发,把UART1接收的数据通过BLE(UART2)直接转发出去
							communicationSend_Struct(&uart1Buff_t,UR2_Enabled);
						}break;
					}			
				}break;
			}
			#endif
			if(0xFE!=gManage_t){								//0XFE不用回复
				uart1SendTxQueueTimeout(&uart1Buff_t);
			}
		}
	}
}
static void uart1Tx_HTask(void *pvParameters)
{	
	stcATBuff uart1TxBuff_t;
    for( ;; ) {	
		uart1ReadTxQueueTimeout(&uart1TxBuff_t);
	}
}
void uart1_TaskInit()
{	
	uart1RecvQueue = xQueueCreate(5, sizeof(stcATBuff));
	if(configParam.ROS_SERIALx != SERIAL1){
		uart1SendQueue = xQueueCreate(5, sizeof(stcATBuff));
		uart1CompleteBinary = xSemaphoreCreateBinary();
		xSemaphoreTake(uart1CompleteBinary, 0);
		#ifndef Custom
		registerComSendCallback(uart1SendTxQueueTimeout,RELAID_UART1);
		#endif
		u1TaskSetDmaSendOver_func(uart1DmaSendOver_callback);			//DMA发送完成注册
		u1SetDmaRead_func(uart1SendRxQueueTimeout);						//DMA读取串口数据注册
		USART1_Init(115200);
		xTaskCreate(uart1Tx_HTask,(const char *)"uart1Tx_HTask",256, NULL,uart1Tx_Pri, NULL);
	}else{
		#ifndef Custom
		rosSetuart1Taskfunc(uart1ToToolsDisable);
		rosSetDmaRead_func(uart1ToToolsSendRxQueueTimeout);
		#endif
	}
	xTaskCreate(uart1Rx_HTask,(const char *)"uart1Rx_HTask",256, NULL,uart1Rx_Pri, NULL);
}