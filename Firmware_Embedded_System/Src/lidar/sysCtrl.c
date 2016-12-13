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

#define LED_SET_ONE 	0x01
#define LED_SET_TWO 	0x02
#define LED_SET_THREE 	0x04

void LED_Control(void* argument);
void Button_Control(void* argument);

extern uint8_t LED_Register_Bits;
extern uint8_t Button_Register_Bits;

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

void Button_Control(void* argument)
{
	uint8_t debounce = 0;

	while(1){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != GPIO_PIN_SET && (debounce & 0x01) != 0x00){
			LED_Register_Bits ^= 0x01;
			Button_Register_Bits |= 0x01;
			debounce &= 0xFE;
		}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != GPIO_PIN_RESET){
			debounce |= 0x01;
			Button_Register_Bits &= 0xFE;
		}
		
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) != GPIO_PIN_SET && (debounce & 0x02) != 0x00){
			LED_Register_Bits ^= 0x02;
			Button_Register_Bits |= 0x02;
			debounce &= 0xFD;
		}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) != GPIO_PIN_RESET){
			debounce |= 0x02;
			Button_Register_Bits &= 0xFD;
		}
		
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != GPIO_PIN_SET && (debounce & 0x04) != 0x00){
			LED_Register_Bits ^= 0x04;
			Button_Register_Bits |= 0x04;
			debounce &= 0xFB;
		}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) != GPIO_PIN_RESET){
			debounce |= 0x04;
			Button_Register_Bits &= 0xFB;
		}

		vTaskDelay(100);
	}

}


#endif