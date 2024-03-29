/**
******************************************************************************
* File Name          : main.c
* Description        : Main program body
******************************************************************************
*
* Copyright (c) 2016 STMicroelectronics International N.V. 
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted, provided that the following conditions are met:
*
* 1. Redistribution of source code must retain the above copyright notice, 
*    this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 3. Neither the name of STMicroelectronics nor the names of other 
*    contributors to this software may be used to endorse or promote products 
*    derived from this software without specific written permission.
* 4. This software, including modifications and/or derivative works of this 
*    software, must execute solely and exclusively on microcontroller or
*    microprocessor devices manufactured by or for STMicroelectronics.
* 5. Redistribution and use of this software other than as permitted under 
*    this license is void and will automatically terminate your rights under 
*    this license. 
*
* THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
* PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
* RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
* SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void example(void* pvParameters);
extern void water(void* pvParameters);
extern void LED_Control(void* argument);

extern void usbDr(void* pvParameters);
extern void serialDr(void* pvParameters);
extern void comInter(void* pvParameters);
extern void sensDriv(void* pvParameters);
extern void photoInt(void* pvParameters);
extern void hBridge(void* pvParameters);
extern void easyStep(void* pvParameters);
extern void sysCtrl(void* pvParameters);
extern void posCtrl(void* pvParameters);

static TaskHandle_t xUsbDrHandle = NULL;
static TaskHandle_t xSerialDrHandle = NULL;
static TaskHandle_t xComInterHandle = NULL;
static TaskHandle_t xSensDrivHandle = NULL;
static TaskHandle_t xPhotoIntHandle = NULL;
static TaskHandle_t xHBridgeHandle = NULL;
static TaskHandle_t xEasyStepHandle = NULL;
static TaskHandle_t xSysCtrlHandle = NULL;
static TaskHandle_t xPosCtrlHandle = NULL;


// Semaphore Declarations:
SemaphoreHandle_t initPosCTRL; // init postion controller
SemaphoreHandle_t startPosCTRL; // start scanning, (postion controller)

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define LED_SET_ONE 00000001
#define LED_SET_TWO 00000010
#define LED_SET_THREE 00000100

uint8_t LED_Register_Bits = 00001010;
//xQueueHandle_t serialInQueue, serialOutQueue;
/* USER CODE END 0 */

int main(void)
{
	
	/* USER CODE BEGIN 1 */
	
	/* USER CODE END 1 */
	
	/* MCU Configuration----------------------------------------------------------*/
	
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	
	/* Configure the system clock */
	SystemClock_Config();
	
	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	MX_ADC_Init();
	MX_USART1_UART_Init();
	MX_USB_DEVICE_Init();
	
	EXTI->RTSR = EXTI_RTSR_TR6;
	
    NVIC_SetPriority(EXTI4_15_IRQn, 0);
  	NVIC_EnableIRQ(EXTI4_15_IRQn);
	/* USER CODE BEGIN 2 */
	
	
	// this queue should be filled in a interrupt using xQueueSendToBackFromISR() function
	/* Create a queue capable of containing 100 char values. */
	serialInQueue = xQueueCreate( 100, sizeof( char ) );
	if (serialInQueue == 0){
        // error
    }
	
	// this queue should be read in a interrupt using xQueueReceiveFromISR() function
	/* Create a queue capable of containing 100 char values. */
	serialOutQueue = xQueueCreate( 100, sizeof( char ) );
	if (serialOutQueue == 0){
        // error
    }
	
	/* USER CODE END 2 */
	
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	
	while (1)
	{
		initPosCTRL = xSemaphoreCreateBinary();
		if(initPosCTRL == NULL){
			// error handler
		}
		startPosCTRL = xSemaphoreCreateBinary();
		if(startPosCTRL == NULL){
			// error handler
		}		
		
		
		// traceanaylzer

		#if SYSCTRL == 1 
		if (xTaskCreate(
						LED_Control,
						"LED_CONT",
						configMINIMAL_STACK_SIZE + 0,
						(void*) NULL,
						taskIDLE_PRIORITY + 1,
						(xTaskHandle*) NULL)
			!= pdPASS ){
				// error handler
			}

		if (xTaskCreate(
						Button_Control,
						"BUT_CONT",
						configMINIMAL_STACK_SIZE + 0,
						(void*) NULL,
						taskIDLE_PRIORITY + 1,
						(xTaskHandle*) NULL)
			!= pdPASS ){
				// error handler
			}
		#endif
		
		#if USBDR == 1
		xTaskCreate(
		usbDr,
		"usbDr",
		configMINIMAL_STACK_SIZE + 0,
		(void*) NULL,
		taskIDLE_PRIORITY + 1,
		(xTaskHandle*) NULL);
		#endif

		#if SERIALDR == 1		
		xTaskCreate(
		serialDr,
		"serialDr",
		configMINIMAL_STACK_SIZE + 0,
		(void*) NULL,
		taskIDLE_PRIORITY + 1,
		(xTaskHandle*) NULL);
		#endif
		
		#if COMINTER == 1
		xTaskCreate(
		comInter,
		"comInter",
		configMINIMAL_STACK_SIZE + 0,
		(void*) NULL,
		taskIDLE_PRIORITY + 1,
		(xTaskHandle*) NULL);
		#endif

		#if SENSDRIV == 1
		xTaskCreate(
		sensDriv,
		"sensDriv",
		configMINIMAL_STACK_SIZE + 0,
		(void*) NULL,
		taskIDLE_PRIORITY + 1,
		(xTaskHandle*) NULL);
		#endif
		
		#if PHOTOINT == 1
		xTaskCreate(
		photoInt,
		"photoInt",
		configMINIMAL_STACK_SIZE + 0,
		(void*) NULL,
		taskIDLE_PRIORITY + 1,
		(xTaskHandle*) NULL);
		#endif
		
		#if HBRIDGE == 1
		xTaskCreate(
		hBridge,
		"hBridge",
		configMINIMAL_STACK_SIZE + 0,
		(void*) NULL,
		taskIDLE_PRIORITY + 1,
		(xTaskHandle*) NULL);
		#endif
		
		#if EASYSTEP == 1
		xTaskCreate(
		easyStep,
		"easyStep",
		configMINIMAL_STACK_SIZE + 0,
		(void*) NULL,
		taskIDLE_PRIORITY + 1,
		(xTaskHandle*) NULL);
		#endif
		
		#if SYSCTRL == 1
		xTaskCreate(
		sysCtrl,
		"sysCtrl",
		configMINIMAL_STACK_SIZE + 0,
		(void*) NULL,
		taskIDLE_PRIORITY + 1,
		(xTaskHandle*) NULL);
		#endif

		#if POSCTRL == 1		
		if(xTaskCreate(
					   posCtrl,
					   "posCtrl",
					   configMINIMAL_STACK_SIZE + 0,
					   (void*) NULL,
					   taskIDLE_PRIORITY + 1,
					   &xPosCtrlHandle)
		   != pdPASS ){
			   // error handler
		   }
		#endif
		
		
		
		
		/* Start scheduler */
		//osKernelStart();
		HAL_GPIO_WritePin (GPIOB, status_led3_Pin|status_led2_Pin|status_led1_Pin, GPIO_PIN_SET);
		vTaskStartScheduler();	// should not get past this function
		for(;;);
		/* USER CODE END WHILE */
		
		/* USER CODE BEGIN 3 */
		
	}
		/* USER CODE END 3 */
		
}
		
		/** System Clock Configuration
		*/
		void SystemClock_Config(void)
		{
			
			RCC_OscInitTypeDef RCC_OscInitStruct;
			RCC_ClkInitTypeDef RCC_ClkInitStruct;
			RCC_PeriphCLKInitTypeDef PeriphClkInit;
			
			/**Initializes the CPU, AHB and APB busses clocks 
			*/
			RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
			RCC_OscInitStruct.HSIState = RCC_HSI_ON;
			RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
			RCC_OscInitStruct.HSICalibrationValue = 16;
			RCC_OscInitStruct.HSI14CalibrationValue = 16;
			RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
			RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
			RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
			RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
			if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
			{
				Error_Handler();
			}
			
			/**Initializes the CPU, AHB and APB busses clocks 
			*/
			RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
				|RCC_CLOCKTYPE_PCLK1;
			RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
			RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
			
			if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
			{
				Error_Handler();
			}
			
			PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
			PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
			PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
			
			if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
			{
				Error_Handler();
			}
			
			/**Configure the Systick interrupt time 
			*/
			HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
			
			/**Configure the Systick 
			*/
			HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
			
			/* SysTick_IRQn interrupt configuration */
			HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
		}
		
		/* USER CODE BEGIN 4 */

		/* USER CODE END 4 */
		
		/**
		* @brief  Period elapsed callback in non blocking mode
		* @note   This function is called  when TIM3 interrupt took place, inside
		* HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
		* a global variable "uwTick" used as application time base.
		* @param  htim : TIM handle
		* @retval None
		*/
		void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
		{
			/* USER CODE BEGIN Callback 0 */
			
			/* USER CODE END Callback 0 */
			if (htim->Instance == TIM3) {
				HAL_IncTick();
			}
			/* USER CODE BEGIN Callback 1 */
			
			/* USER CODE END Callback 1 */
		}
		
		/**
		* @brief  This function is executed in case of error occurrence.
		* @param  None
		* @retval None
		*/
		void Error_Handler(void)
		{
			/* USER CODE BEGIN Error_Handler */
			/* User can add his own implementation to report the HAL error return state */
			while(1) 
			{
			}
			/* USER CODE END Error_Handler */ 
		}
		
#ifdef USE_FULL_ASSERT
		
		/**
		* @brief Reports the name of the source file and the source line number
		* where the assert_param error has occurred.
		* @param file: pointer to the source file name
		* @param line: assert_param error line source number
		* @retval None
		*/
		void assert_failed(uint8_t* file, uint32_t line)
		{
			/* USER CODE BEGIN 6 */
			/* User can add his own implementation to report the file name and line number,
			ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
			/* USER CODE END 6 */
			
		}
		
#endif
		
		/**
		* @}
		*/ 
		
		/**
		* @}
		*/ 
		
		/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
		