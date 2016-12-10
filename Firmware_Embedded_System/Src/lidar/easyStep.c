/**
 * @file easyStep.c
 * @author Author here
 * @date 26 nov 2016
 * @brief bla bla bla
 *
 */
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lidarDefaultHeader.h"

#if EASYSTEP == 1
// HAL_GPIO_WritePin(EASY_ENABLE_GPIO_Port, EASY_ENABLE_Pin, ....);	/* ENABLE		*/ 		// constant?
// HAL_GPIO_WritePin(EASY_RESET_GPIO_Port, EASY_RESET_Pin, ....);		/* RESET		*/ 		// constant?
// HAL_GPIO_WritePin(EASY_MS2_GPIO_Port, EASY_MS2_Pin, ....);			/* MS2 Pin		*/ 		// depends on setting
// HAL_GPIO_WritePin(EASY_MS1_GPIO_Port, EASY_MS1_Pin, ....);			/* MS1 pin     	*/ 		// depends on setting
// HAL_GPIO_WritePin(EASY_DIR_GPIO_Port, EASY_DIR_Pin, ....);			/* Direction 	*/
// HAL_GPIO_WritePin(EASY_STEP_GPIO_Port, EASY_STEP_Pin, ....);		/* Rotate motor */
// HAL_GPIO_WritePin(EASY_SLEEP_GPIO_Port, EASY_SLEEP_Pin, ....);		/* SLEEP		*/ 		// constant?
/*
#define EASY_ENABLE_Pin GPIO_PIN_10
#define EASY_ENABLE_GPIO_Port GPIOB
#define EASY_RESET_Pin GPIO_PIN_11
#define EASY_RESET_GPIO_Port GPIOB
#define EASY_MS2_Pin GPIO_PIN_12
#define EASY_MS2_GPIO_Port GPIOB
#define EASY_MS1_Pin GPIO_PIN_13
#define EASY_MS1_GPIO_Port GPIOB
#define EASY_DIR_Pin GPIO_PIN_14
#define EASY_DIR_GPIO_Port GPIOB
#define EASY_STEP_Pin GPIO_PIN_15
#define EASY_STEP_GPIO_Port GPIOB
#define EASY_SLEEP_Pin GPIO_PIN_8
#define EASY_SLEEP_GPIO_Port GPIOA
#define photointerrupter1_Pin GPIO_PIN_8
#define photointerrupter1_GPIO_Port GPIOB
#define photointerrupter2_Pin GPIO_PIN_9
#define photointerrupter2_GPIO_Port GPIOB
*/

#define CCW GPIO_PIN_RESET
#define CW GPIO_PIN_SET

struct EASYSTEP{
	int dir;
	int stepsize;
	int speed;
};


extern struct EASYSTEP easystep;

/**
 * @brief      Driver for the easystepper, Will receive 'step up' or 'step down' signals
 *
 * @param      pvParameters  The pv parameters
 */
void easyStep(void* pvParameters) {	
	// ulTaskNotifyTake( pdTRUE,  portMAX_DELAY); /* reset notivications to 0 on exit of function, max delay BUT WHAT HAPPENDS IF IT OVERFLOWS??? HOW OT PREVENT!!! */

	/* *** START INIT *** */
	HAL_GPIO_WritePin(EASY_ENABLE_GPIO_Port, EASY_ENABLE_Pin, GPIO_PIN_RESET);			/* ENABLE		*/ 		// constant?
	HAL_GPIO_WritePin(EASY_RESET_GPIO_Port, EASY_RESET_Pin, GPIO_PIN_SET);				/* RESET		*/ 		// constant?
	HAL_GPIO_WritePin(EASY_SLEEP_GPIO_Port, EASY_SLEEP_Pin, GPIO_PIN_SET);				/* SLEEP		*/ 		// constant?

	switch(easystep.stepsize){
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
		

	while(1) 
	{
		/* suspend until next command/ notification */
		ulTaskNotifyTake( pdTRUE,  portMAX_DELAY); /* reset notivications to 0 on exit of function, max delay BUT WHAT HAPPENDS IF IT OVERFLOWS??? HOW OT PREVENT!!! */
		if (1 == easystep.dir){ // Clockwise
			if(HAL_GPIO_ReadPin(photointerrupter1_GPIO_Port, photointerrupter1_Pin) == 0){ // het signaal wordt naar nul getrokken, dus hij ziet licht, dus er zit niets tussen
				HAL_GPIO_WritePin(EASY_DIR_GPIO_Port, EASY_DIR_Pin, GPIO_PIN_SET);			/* Direction 	*/
				HAL_GPIO_WritePin(EASY_STEP_GPIO_Port, EASY_STEP_Pin, GPIO_PIN_SET);		/* Rotate motor */
				vtaskDelay(easystep.speed);
				HAL_GPIO_WritePin(EASY_STEP_GPIO_Port, EASY_STEP_Pin, GPIO_PIN_RESET);		/* Rotate motor */
			}
			else{
				// too far, dont move
			}
		}
		else if(-1 == easystep.dir){ // Counter clockwise
			if(HAL_GPIO_ReadPin(photointerrupter2_GPIO_Port, photointerrupter2_Pin) == 0){// het signaal wordt naar nul getrokken, dus hij ziet licht, dus er zit niets tussen
				HAL_GPIO_WritePin(EASY_DIR_GPIO_Port, EASY_DIR_Pin, GPIO_PIN_RESET);			/* other Direction 	*/
				HAL_GPIO_WritePin(EASY_STEP_GPIO_Port, EASY_STEP_Pin, GPIO_PIN_SET);		/* Rotate motor */
				vtaskDelay(easystep.speed);
				HAL_GPIO_WritePin(EASY_STEP_GPIO_Port, EASY_STEP_Pin, GPIO_PIN_RESET);		/* Rotate motor */
			}
			else{
				// too far dont move
			}
		}
		xTaskNotifyGive( xPosCtrlHandle ); /* resume posCTRL */
	}
}


#undef CCW
#undef CW

#endif