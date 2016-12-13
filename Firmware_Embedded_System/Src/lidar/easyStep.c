#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lidarDefaultHeader.h"

#if EASYSTEP == 1
// HAL_GPIO_WritePin(EASY_ENABLE_GPIO_Port, EASY_ENABLE_Pin, ....);	/* ENABLE		*/ 		// constant?


#define CCW GPIO_PIN_RESET
#define CW GPIO_PIN_SET

int speed = 0;
void easyInit(char stepSize, char _speed){
	speed = _speed;

	/* *** START INIT *** */
	HAL_GPIO_WritePin(EASY_ENABLE_GPIO_Port, EASY_ENABLE_Pin, GPIO_PIN_RESET);			/* ENABLE		*/ 		// constant?
	HAL_GPIO_WritePin(EASY_RESET_GPIO_Port, EASY_RESET_Pin, GPIO_PIN_SET);				/* RESET		*/ 		// constant?
	HAL_GPIO_WritePin(EASY_SLEEP_GPIO_Port, EASY_SLEEP_Pin, GPIO_PIN_SET);				/* SLEEP		*/ 		// constant?

	switch(stepSize){
		case 1: /* 1:8 Eighth step*/
			HAL_GPIO_WritePin(EASY_MS1_GPIO_Port, EASY_MS1_Pin, GPIO_PIN_SET);			/* MS1 pin     	*/ 		// depends on setting
			HAL_GPIO_WritePin(EASY_MS2_GPIO_Port, EASY_MS2_Pin, GPIO_PIN_SET);			/* MS2 Pin		*/ 		// depends on setting
		break;
		case 2: /* 1:4 Quarter step*/
			HAL_GPIO_WritePin(EASY_MS1_GPIO_Port, EASY_MS1_Pin, GPIO_PIN_RESET);		/* MS1 pin     	*/ 		// depends on setting
			HAL_GPIO_WritePin(EASY_MS2_GPIO_Port, EASY_MS2_Pin, GPIO_PIN_SET);			/* MS2 Pin		*/ 		// depends on setting
		break;
		case 3: /* 1:2 Half step*/
			HAL_GPIO_WritePin(EASY_MS1_GPIO_Port, EASY_MS1_Pin, GPIO_PIN_SET);			/* MS1 pin     	*/ 		// depends on setting
			HAL_GPIO_WritePin(EASY_MS2_GPIO_Port, EASY_MS2_Pin, GPIO_PIN_RESET);		/* MS2 Pin		*/ 		// depends on setting
		break;
		case 4: /* 1:1 Full step*/
			HAL_GPIO_WritePin(EASY_MS1_GPIO_Port, EASY_MS1_Pin, GPIO_PIN_RESET);		/* MS1 pin     	*/ 		// depends on setting
			HAL_GPIO_WritePin(EASY_MS2_GPIO_Port, EASY_MS2_Pin, GPIO_PIN_RESET);		/* MS2 Pin		*/ 		// depends on setting
		break;
		default:
			Error_Handler();
	}
}

void easyMove(char direction) {	
	if (1 == direction){ // Clockwise
		if(HAL_GPIO_ReadPin(photointerrupter1_GPIO_Port, photointerrupter1_Pin) == 0){ // het signaal wordt naar nul getrokken, dus hij ziet licht, dus er zit niets tussen
			HAL_GPIO_WritePin(EASY_DIR_GPIO_Port, EASY_DIR_Pin, GPIO_PIN_SET);			/* Direction 	*/
			HAL_GPIO_WritePin(EASY_STEP_GPIO_Port, EASY_STEP_Pin, GPIO_PIN_SET);		/* Rotate motor */
			vTaskDelay(speed);
			HAL_GPIO_WritePin(EASY_STEP_GPIO_Port, EASY_STEP_Pin, GPIO_PIN_RESET);		/* Rotate motor */
		}
		else{
			// too far, dont move
		}
	}
	else if(-1 == direction){ // Counter clockwise
		if(HAL_GPIO_ReadPin(photointerrupter2_GPIO_Port, photointerrupter2_Pin) == 0){// het signaal wordt naar nul getrokken, dus hij ziet licht, dus er zit niets tussen
			HAL_GPIO_WritePin(EASY_DIR_GPIO_Port, EASY_DIR_Pin, GPIO_PIN_RESET);			/* other Direction 	*/
			HAL_GPIO_WritePin(EASY_STEP_GPIO_Port, EASY_STEP_Pin, GPIO_PIN_SET);		/* Rotate motor */
			vTaskDelay(speed);
			HAL_GPIO_WritePin(EASY_STEP_GPIO_Port, EASY_STEP_Pin, GPIO_PIN_RESET);		/* Rotate motor */
		}
		else{
			// too far dont move
		}
	}
	else{
		// da fuck?
	}
	
}


#undef CCW
#undef CW

#endif