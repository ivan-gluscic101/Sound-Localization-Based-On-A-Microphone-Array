#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"

/* Queue handles (defined in task_manager.c, used by stm32g4xx_it.c and tasks) */
extern osMessageQId  queueDmaEventHandle;  /* DMA ISR  -> ACQ_Task  */
extern QueueHandle_t queueSnapshotHandle;  /* ACQ_Task -> LOC_Task  */
extern QueueHandle_t queueResultHandle;    /* LOC_Task -> UART_Task */

/* Create queues and FreeRTOS tasks. Call before osKernelStart(). */
void app_tasks_init(void);

#endif /* TASK_MANAGER_H */
