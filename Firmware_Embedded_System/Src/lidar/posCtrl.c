/**
 * @file posCtrl.c
 * @author Ruben IJlst
 * @date 06 Dec 2016
 * @brief bla bla bla
 *
 */


/* **** CONFIG *** */
#define stepSpeed 1 // tijd dat de motor bezig is om 1 stap te zetten
#define stepDiv = 4 /* can be 1:8, 1:4, 1:2, 1:1 */
 /* *** end config */


#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "lidarDefaultHeader.h"

#if POSCTRL == 1
/**
 * @brief       This function remembers the posiotion of the head. In combination with the input from the photo interruptor,
	It will make sure that the head doesnt turn to far, and that it can be initialized.
 *
 * @param      pvParameters  The pv parameters
 */


/* 	if motor driver is set to 1:8 then stepsize should be 1 */
#if stepDiv == 1   
#define STEPSIZE 8 
#define SPEED 0
#endif

/*  if motor driver is set to 1:4 then stepsize should be 2 */ 
#if stepDiv == 2
#define STEPSIZE 4
#define SPEED 0
#endif

/* 	if motor driver is set to 1:2 then stepsize should be 4 */
#if stepDiv == 4
#define STEPSIZE 2
#define SPEED 0
#endif

/* 	if motor driver is set to 1:1 then stepsize should be 8 */
#if stepDiv == 8
#define STEPSIZE 1
#define SPEED 0
#endif

/* 	if H-bridge is used, then stepsize should be 8 			*/
#if stepDiv == hbridge
#define STEPSIZE 8
#define SPEED 0
#endif

struct EASYSTEP easyStruct;


/* ga communiceren met globale variabelen */

extern SemaphoreHandle_t initPosCTRL; // init postion controller
extern SemaphoreHandle_t startPosCTRL; // start scanning, (postion controller)
uint32_t position = 0;

enum STATE{ERROR, INIT, STEP, START_MEASUREMENT, RESET};

void move(int dir, int amountOfSteps){
	for(; amountOfSteps; amountOfSteps--){
		easyStruct.dir = dir;
		xTaskNotifyGive( xEasyStepHandle );
		ulTaskNotifyTake( pdTRUE,  portMAX_DELAY);
	}
}


void posCtrl(void* pvParameters) {
 
	easyStruct.dir = 0;
	easyStruct.stepsize = STEPSIZE;
	easyStruct.speed = SPEED;



	/* start Motor Driver */


	enum STATE state = RESET;
	while(1) {
		switch(state){
	
		/*	Reset: wait for command to start initializing.	
			if init command is received, initialize the motor
		*/
		case RESET:
			// if( xEasyStepHandle != NULL )
		 //    {
		 //        vTaskDelete( xEasyStepHandle );
		 //    }

		// 	while(initPosCTRL != NULL) vTaskDelay(10); // check if semaphore is created
		// 	while(startPosCTRL != NULL)vTaskDelay(10); // check if semaphore is created
		// 	if(xSemaphoreTake(, 100) == pdTRUE){ /* wait for init command */
		// 		state = INIT; /* go to state init */
		// 	}
		// 	else{
		// 		// duurtlang
		// 	}
		// break;
		
		 //    if(posControlStruct.init <= 0){
		 //    	taskYIELD(); // or // vTaskDelay(5);
		 //    }
		 //    else{
		 //    	posControlStruct.init = 0;
		 //    	state = INIT;
			// }
		case INIT:

			/* Start motor driver */
			// This task should have a high(er) priority than the parent task
			// xTaskCreate(
			// 		easyStep,
			// 		"easyStep",
			// 		configMINIMAL_STACK_SIZE + 0,
			// 		&easyStruct
			// 		(void*) NULL,
			// 		taskIDLE_PRIORITY + 1,
			// 		&xEasyStepHandle);

			// taskYIELD();

			easyInit(easyStruct);
			/* Initialize motor */
			while(HAL_GPIO_ReadPin(photointerrupter2_GPIO_Port, photointerrupter2_Pin) == 0){
				easyStruct.dir = -1;
				easyStep(easyStruct);
				// ulTaskNotifyGive(xEasyStepHandle);
				// ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			}

			postion = 0;

		    if(posControlStruct.start <= 0){
		    	taskYIELD(); // or // vTaskDelay(5);
		    }
		    else{
		    	posControlStruct.start--;
		    	state = STEP;
			}



		break;
		case STEP:
			// geef moter driver een schop OF MEERDERE moet ik nog nakijken
			easyStruct.dir = 1;
			for(int i = 0; i < 5; i++){
				easyStep(easyStruct);
				position += stepSize;		/* keep track of the current position */
				vTaskDelay(stepSpeed);  	/* Give the motor time to move */
			}
			state = START_MEASUREMENT; 	/* Next state START_MEASUREMENT */
		break;
		case START_MEASUREMENT:
			// geef longRangeSensorDriver een schop (positie mee geven)
			// geef shortRangeSensorDriver een schop (positie mee geven)
			// wait semaphore/signal from sensors
			

			// retrieve raw data from the short range sensor and store it in a register
			// retrieve raw data from the long range sensor and store it in a register
			
			if ( position >= 752)
				state = INIT; // return motor to starting position
			else if(HAL_GPIO_ReadPin(photointerrupter1_GPIO_Port, photointerrupter1_Pin) == 0){ // het signaal wordt naar nul getrokken, dus hij ziet licht, dus er zit niets tussen
				state = ERROR;
			}
			else
				state = STEP;
		break;
		case ERROR:
			Error_Handler();
			state = RESET;
		break;
		}
	}
}
#undef stepSizee
#undef stepSpeede
#undef stepDiv1e

#endif



210 graden
1.8 per stap

180 100
30 


helestappen 210.6 = 117 hele stappen

1		2		4		8
1.8		0.9		0.45	0.225


1.389	2.78	5.56	11.11

				^
				en dan elke 5 stappen,dus 5*0.45 =  2.25 graden per meting
				dus 210/2.25 afgerond 94 metingen
				94*8 = 752 stappen