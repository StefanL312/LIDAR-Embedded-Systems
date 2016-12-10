/**
 * @file example.c
 * @author Author here
 * @date 26 nov 2016
 * @brief example file
 *
 */

#include <stdio.h>
#include "stm32f0xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
			  
#include "lidarDefaultHeader.h"

#if EXAMPLE == 1
uint8_t uxHighWaterMark = 0;

/**
 * @brief      Example
 *
 * @param      pvParameters  The pv parameters
 */
 extern TaskHandle_t test;

/* StartDefaultTask function */
void example(void* pvParameters)
{
  /* init code for USB_DEVICE */

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		vTaskDelay(250);
		HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_2);
  }
  /* USER CODE END StartDefaultTask */
}


void water(void* pvParameters){
	
	
	while(1){
		uxHighWaterMark = uxTaskGetStackHighWaterMark( test );
		vTaskDelay(100);
	}
}

#endif