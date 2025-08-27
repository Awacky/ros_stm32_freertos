
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "microros_tasks.h"

typedef StaticTask_t osStaticThreadDef_t;

osThreadId_t defaultTaskHandle;

uint32_t defaultTaskBuffer[3000];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t sysLedTaskHandle;
const osThreadAttr_t sysLedTask_attributes = {
  .name = "sysLedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};

void StartDefaultTask(void *argument);
void sysLedTask(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void MX_FREERTOS_Init(void) {
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  sysLedTaskHandle  = osThreadNew(sysLedTask,       NULL, &sysLedTask_attributes);
}

void StartDefaultTask(void *argument){
  run_starrobotSTM_node();
}

void sysLedTask(void *argument){
  while(1){
     HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
     osDelay(200);
  }
}


