

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif
#ifndef CMSIS_device_header
#define CMSIS_device_header "stm32f4xx.h"
#endif /* CMSIS_device_header */

/***************************************************************************************************************/
/*                                        FreeRTOS������������ѡ��                                              */
/***************************************************************************************************************/
#define configENABLE_FPU                          	0
#define configENABLE_MPU                          	0

#define configUSE_PREEMPTION					  	1                       //1ʹ����ռʽ�ںˣ�0ʹ��Э��
#define configUSE_TIME_SLICING					  	1						//1ʹ��ʱ��Ƭ����(Ĭ��ʽʹ�ܵ�)
#define configUSE_PORT_OPTIMISED_TASK_SELECTION		0                       //1�������ⷽ����ѡ����һ��Ҫ���е�����//1
																			//һ����Ӳ������ǰ����ָ������ʹ�õ�
																			//MCUû����ЩӲ��ָ��Ļ��˺�Ӧ������Ϊ0��
#define configUSE_TICKLESS_IDLE					    0                       //1���õ͹���ticklessģʽ 
#define configUSE_QUEUE_SETS						1                       //Ϊ1ʱ���ö��� 
#define configCPU_CLOCK_HZ                        	( SystemCoreClock )     //CPUƵ��
#define configTICK_RATE_HZ                        	((TickType_t)1000)      //ʱ�ӽ���Ƶ�ʣ���������Ϊ1000�����ھ���1ms
#define configMAX_PRIORITIES                      	( 32 )                  //��ʹ�õ�������ȼ�                
#define configMINIMAL_STACK_SIZE                  	((uint16_t)130)         //��������ʹ�õĶ�ջ��С
#define configMAX_TASK_NAME_LEN                  	( 16 )                  //���������ַ�������

#define configIDLE_SHOULD_YIELD						1                       //Ϊ1ʱ�����������CPUʹ��Ȩ������ͬ���ȼ����û����� N
#define configUSE_TASK_NOTIFICATIONS   				1                       //Ϊ1ʱ��������֪ͨ���ܣ�Ĭ�Ͽ��� N
#define configUSE_MUTEXES                			1
#define configQUEUE_REGISTRY_SIZE					8                       //��Ϊ0ʱ��ʾ���ö��м�¼�������ֵ�ǿ��Լ�¼�Ķ��к��ź��������Ŀ��

#define configCHECK_FOR_STACK_OVERFLOW				0                       //����0ʱ���ö�ջ�����⹦�ܣ����ʹ�ô˹��� N
																			//�û������ṩһ��ջ������Ӻ��������ʹ�õĻ�
																			//��ֵ����Ϊ1����2����Ϊ������ջ�����ⷽ����
#define configUSE_RECURSIVE_MUTEXES					1                       //Ϊ1ʱʹ�õݹ黥���ź���
#define configUSE_MALLOC_FAILED_HOOK			 	0                       //1ʹ���ڴ�����ʧ�ܹ��Ӻ���
#define configUSE_APPLICATION_TASK_TAG			  	0                       //
#define configUSE_COUNTING_SEMAPHORES   			1						//Ϊ1ʱʹ�ü����ź���

#define configMESSAGE_BUFFER_LENGTH_TYPE          	size_t					//N

/***************************************************************************************************************/
/*                                FreeRTOS���ڴ������й�����ѡ��                                                */
/***************************************************************************************************************/
#define configSUPPORT_STATIC_ALLOCATION           	1                       //��̬�����ڴ� N 1
#define configSUPPORT_DYNAMIC_ALLOCATION          	1                       //֧�ֶ�̬�ڴ�����
#ifdef STM32F10X_HD
	#define configTOTAL_HEAP_SIZE					((size_t)(40*1024))    	//ϵͳ�����ܵĶѴ�С
#else 
	#define configTOTAL_HEAP_SIZE					((size_t)(60*1024))    	//ϵͳ�����ܵĶѴ�С 74
#endif

/***************************************************************************************************************/
/*                                FreeRTOS�빳�Ӻ����йص�����ѡ��                                              */
/***************************************************************************************************************/
#define configUSE_IDLE_HOOK							1                       //1��ʹ�ÿ��й��ӣ�0����ʹ��
#define configUSE_TICK_HOOK							0                       //1��ʹ��ʱ��Ƭ���ӣ�0����ʹ��

/***************************************************************************************************************/
/*                                FreeRTOS������ʱ�������״̬�ռ��йص�����ѡ��                                 */
/***************************************************************************************************************/
#define configGENERATE_RUN_TIME_STATS	            0                       //Ϊ1ʱ��������ʱ��ͳ�ƹ���
#define configUSE_16_BIT_TICKS						0                       //ϵͳ���ļ����������������ͣ�
																			//1��ʾΪ16λ�޷������Σ�0��ʾΪ32λ�޷�������
#define configUSE_TRACE_FACILITY					1
#define configUSE_STATS_FORMATTING_FUNCTIONS	    1                       //���ͬʱΪ1ʱ���������3������
																			//prvWriteNameToBuffer(),vTaskList(),
																			//vTaskGetRunTimeStats()

/***************************************************************************************************************/
/*                                FreeRTOS��Э���йص�����ѡ��                                                  */
/***************************************************************************************************************/
#define configUSE_CO_ROUTINES                    	0
#define configMAX_CO_ROUTINE_PRIORITIES          	( 2 )

/***************************************************************************************************************/
/*                                FreeRTOS�������ʱ���йص�����ѡ��                                            */
/***************************************************************************************************************/
#define configUSE_TIMERS                         	1
#define configTIMER_TASK_PRIORITY		        	2//(configMAX_PRIORITIES-1)        //�����ʱ�����ȼ� 2
#define configTIMER_QUEUE_LENGTH		        	5                               //�����ʱ�����г��� 10 
#define configTIMER_TASK_STACK_DEPTH	        	256//(configMINIMAL_STACK_SIZE*2)    //�����ʱ�������ջ��С 256

/***************************************************************************************************************/
/*                                FreeRTOS��ѡ��������ѡ��                                                      */
/***************************************************************************************************************/
#define INCLUDE_xTaskGetSchedulerState       		1
#define INCLUDE_vTaskPrioritySet             		1
#define INCLUDE_uxTaskPriorityGet            		1
#define INCLUDE_vTaskDelete                  		1
#define INCLUDE_vTaskCleanUpResources        		0	 //1
#define INCLUDE_vTaskSuspend                 		1
#define INCLUDE_vTaskDelayUntil              		1
#define INCLUDE_vTaskDelay                   		1
#define INCLUDE_xTimerPendFunctionCall       		1
#define INCLUDE_xQueueGetMutexHolder         		1
#define INCLUDE_uxTaskGetStackHighWaterMark  		1
#define INCLUDE_xTaskGetCurrentTaskHandle    		1
#define INCLUDE_eTaskGetState                		1

/* CMSIS-RTOS V2 flags */
#define configUSE_OS2_THREAD_SUSPEND_RESUME  1
#define configUSE_OS2_THREAD_ENUMERATE       1
#define configUSE_OS2_EVENTFLAGS_FROM_ISR    1
#define configUSE_OS2_THREAD_FLAGS           1
#define configUSE_OS2_TIMER                  1
#define configUSE_OS2_MUTEX                  1

#define USE_FreeRTOS_HEAP_4

/***************************************************************************************************************/
/*                                FreeRTOS���ж��йص�����ѡ��                                                  */
/***************************************************************************************************************/
#ifdef __NVIC_PRIO_BITS
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/***************************************************************************************************************/
/*                                ������������ѡ��                                                              */
/***************************************************************************************************************/
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 0

#endif 
