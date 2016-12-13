/**
 * @file sysCtrl.c
 * @author Author here
 * @date 26 nov 2016
 * @brief bla bla bla
 *
 */
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lidarDefaultHeader.h"

#if SYSCTRL == 1

#define LED_SET_ONE 	00000001
#define LED_SET_TWO 	00000010
#define LED_SET_THREE 	00000100

void LED_Control(void* argument);
void Button_Control(void* argument);

extern uint8_t LED_Register_Bits;
extern uint8_t Button_Status;
/*
	Set Pinouts for LEDs with status of pointer argument.
*/
void LED_Control(void* argument)
{
	uint8_t slowdown = 0;

	while(1){
		HAL_GPIO_WritePin(GPIOB, status_led1_Pin, ((LED_Register_Bits & LED_SET_ONE)!=0)?GPIO_PIN_RESET:GPIO_PIN_SET);
			
		HAL_GPIO_WritePin(GPIOB, status_led2_Pin, ((LED_Register_Bits & LED_SET_TWO)!=0)?GPIO_PIN_RESET:GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, status_led3_Pin, ((LED_Register_Bits & LED_SET_THREE)!=0)?GPIO_PIN_RESET:GPIO_PIN_SET);

		if(slowdown == 10){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
			slowdown = 0;
		}

		slowdown++;
		vTaskDelay(50);
	}

}

/*Set status for the buttons.*/
void Button_Control(void* argument)
{
	uint8_t debounce = 0;

	while(1){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != GPIO_PIN_SET && (debounce & 00000001) != 0){
			LED_Register_Bits ^= 00000001;
			Button_Status ^= 00000001;
			debounce &= 00000001;
		}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != GPIO_PIN_RESET){
			debounce |= 00000001;
		}
		
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) != GPIO_PIN_SET && (debounce & 00000010) != 0){
			LED_Register_Bits ^= 00000010;
			Button_Status ^= 00000010;
		}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != GPIO_PIN_RESET){
			debounce |= 00000010;
		}
		
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != GPIO_PIN_SET && (debounce & 00000100) != 0){
			LED_Register_Bits ^= 00000100;
			Button_Status ^= 00000100;
		}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != GPIO_PIN_RESET){
			debounce |= 00000100;
		}

		vTaskDelay(100);
	}

}


#endif