/**
  ******************************************************************************
  * @file    main.c
  * @brief   Startup: clock -> custom LL drivers -> I2C/IMU -> tasks -> scheduler
  *
  * Peripherals are initialized by the custom LL drivers in CustomDriverSet,
  * not by CubeMX MX_*_Init. Do not regenerate this file from CubeMX.
  ******************************************************************************
  */
#include "main.h"
#include "cmsis_os.h"

#include "gpio_driver.h"     /* Custom_GPIO_Init */
#include "dma_driver.h"      /* Custom_DMA_Init */
#include "adc_driver.h"      /* Custom_ADC_Init, Custom_ACQ_Start */
#include "uart_driver.h"     /* Custom_UART4_Init */
#include "timer_driver.h"    /* Custom_TIM8_Init */
#include "task_manager.h"    /* app_tasks_init */
#include "app_init.h"        /* app_init (I2C + IMU + diagnostics) */

void SystemClock_Config(void);

/* Application entry point. */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Peripherals via custom LL drivers (order: GPIO clocks first, then the rest). */
  Custom_GPIO_Init();
  Custom_DMA_Init();
  Custom_ADC_Init();
  Custom_UART4_Init();
  Custom_TIM8_Init();

  /* I2C LL + IMU + diagnostics + gyro calibration. */
  app_init();

  /* Create RTOS objects (queues/tasks) before starting acquisition, since the
   * DMA ISR posts to queueDmaEventHandle; Custom_ACQ_Start() runs after. */
  app_tasks_init();
  Custom_ACQ_Start();

  osKernelStart();

  /* Scheduler now has control; never reached. */
  while (1)
  {
  }
}

/* System clock: 170 MHz from HSI via PLL. */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(170000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* HAL time base tick (TIM6 -> HAL_IncTick). */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}

/* FreeRTOS stack overflow hook: halt. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask; (void)pcTaskName;
  __disable_irq();
  while (1) {}
}

/* FreeRTOS malloc-failed hook: halt. */
void vApplicationMallocFailedHook(void)
{
  __disable_irq();
  while (1) {}
}

/* Fatal error trap. */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/* Reports the file and line of a failed assert_param. */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
