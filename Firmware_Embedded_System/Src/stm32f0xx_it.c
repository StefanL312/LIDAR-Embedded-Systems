/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lidarDefaultHeader.h"
// #include "main.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;

extern TIM_HandleTypeDef htim3;

extern uint8_t LED_Register_Bits;


/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
//void SVC_Handler(void)
//{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
//}

/**
* @brief This function handles Pendable request for system service.
*/
//void PendSV_Handler(void)
//{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
//}

/*
Buttons GPIOA Pin: 4 5 6
EXTI4: Resumes Task blabla to do blabla

EXTI5: Resumes Task blabla to do blabla

EXTI6: Resumes Task blabla to do blabla

EXTI8: Interrupt of Photointerrupter 1

EXTI9: Interrupt of Photointerrupter 2

*/
void EXTI4_15_IRQHandler()
{
  //if (EXTI_GetITStatus(EXTI_Line4) != RESET){
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != GPIO_PIN_SET){
    LED_Register_Bits ^= 00000001;
  }

  //if (EXTI_GetITStatus(EXTI_Line5) != RESET){
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) != GPIO_PIN_SET){
    LED_Register_Bits ^= 00000010;
  }

  //if (EXTI_GetITStatus(EXTI_Line6) != RESET){
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != GPIO_PIN_SET){
    LED_Register_Bits ^= 00000100;
  }

  /*if (EXTI_GetITStatus(EXTI_Line8) != RESET){
    xTaskResume();
    EXTI_ClearITPendingBit(EXTI_Line6);
  }

  if (EXTI_GetITStatus(EXTI_Line9) != RESET){
    xTaskResume();
    EXTI_ClearITPendingBit(EXTI_Line6);
  }*/
}

/**
* @brief This function handles System tick timer.
*/

//void SysTick_Handler(void)
//{
//  /* USER CODE BEGIN SysTick_IRQn 0 */
//
//  /* USER CODE END SysTick_IRQn 0 */
//  HAL_SYSTICK_IRQHandler();
//  /* USER CODE BEGIN SysTick_IRQn 1 */
//
//  /* USER CODE END SysTick_IRQn 1 */
//}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles USB global Interrupt / USB wake-up interrupt through EXTI line 18.
*/
void USB_IRQHandler(void)
{
  /* USER CODE BEGIN USB_IRQn 0 */

  /* USER CODE END USB_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_IRQn 1 */

  /* USER CODE END USB_IRQn 1 */
}

void USART1_IRQHandler(void){
	BaseType_t xTaskWokenByReceive = pdFALSE;
	BaseType_t xHigherPriorityTaskWoken;
	char rxBuffer, txBuffer;
	if((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
	{
		// pull from serialOutQueue and store in TDR (Transmit data register)
		if(xQueueReceiveFromISR( serialOutQueue, ( void * ) &txBuffer, &xTaskWokenByReceive) == pdTRUE)
		{	
			USART1->TDR = (uint8_t) txBuffer;
			
		}
		USART1->ICR |= USART_ICR_TCCF;	// clear TC flag

		if( xTaskWokenByReceive != pdFALSE )
	    {
	        taskYIELD();
	    }
        
	}
	else if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{ 
		//data available in RDR register. Push it in the serialInQueue
		rxBuffer = (char) (USART1->RDR);
		xQueueSendFromISR( serialInQueue, &rxBuffer, &xHigherPriorityTaskWoken );	

		USART1->RQR |= USART_RQR_RXFRQ;		// clear RXNE flag

		/* Now the buffer is empty we can switch context if necessary. */
	    if( xHigherPriorityTaskWoken )
	    {
	        /* Actual macro used here is port specific. */
	        taskYIELD();
	    }
	}
	else if((USART1->ISR & USART_ISR_PE) == USART_ISR_PE)
	{
		USART1->ICR |= USART_ICR_PECF;
		// parity error
	}
	else if((USART1->ISR & USART_ISR_FE) == USART_ISR_FE)
	{
		USART1->ICR |= USART_ICR_FECF;
		// Framing error
	}
	else if((USART1->ISR & USART_ISR_NE) == USART_ISR_NE)
	{
		USART1->ICR |= USART_ICR_NCF;
		// uhhhm
	}
	else if((USART1->ISR & USART_ISR_ORE) == USART_ISR_ORE)
	{
		USART1->ICR |= USART_ICR_ORECF;
		// overrun error
	}
	else if((USART1->ISR & USART_ISR_RTOF) == USART_ISR_RTOF)
	{
		USART1->ICR |= USART_ICR_RTOCF;
		// receiver timeout error
	}
	else
	{
		NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
