/**
 * @file posCtrl.c
 * @author Ruben IJlst
 * @date 06 Dec 2016
 * @brief bla bla bla
 *
 */


/* **** CONFIG *** */
#define stepSpeed 1 // tijd dat de motor bezig is om 1 stap te zetten
#define stepDiv = 1 /* can be 1:8, 1:4, 1:2, 1:1 */
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

struct EASYSTEP{
	int dir;
	int stepsize;
	int speed;
}easystep;


/* ga communiceren met globale variabelen */

extern SemaphoreHandle_t initPosCTRL; // init postion controller
extern SemaphoreHandle_t startPosCTRL; // start scanning, (postion controller)
uint32_t position = 0;

enum STATE{ERROR, INIT, STEP, START_MEASUREMENT, RESET};

void move(int dir, int amountOfSteps){
	for(; amountOfSteps; amountOfSteps--){
		easytep.dir = dir;
		xTaskNotifyGive( xEasyStepHandle );
		ulTaskNotifyTake( pdTRUE,  portMAX_DELAY);
	}
}

 void posCtrl(void* pvParameters) {
 }
	easystep.dir = 0;
	easystep.stepsize = STEPSIZE;
	easystep.speed = SPEED;



	/* start Motor Driver */


	enum STATE state = RESET;
	while(1) {
		switch(state){
	
		/*	Reset: wait for command to start initializing.	
			if init command is received, initialize the motor
		*/
		case RESET:
			if( xEasyStepHandle != NULL )
		    {
		        vTaskDelete( xEasyStepHandle );
		    }

			while(initPosCTRL != NULL) vTaskDelay(10); // check if semaphore is created
			while(startPosCTRL != NULL)vTaskDelay(10); // check if semaphore is created
			if(xSemaphoreTake(initPosCTRL, 100) == pdTRUE){ /* wait for init command */
				state = INIT; /* go to state init */
			}
			else{
				// duurtlang
			}
		break;
		case INIT:

			/* Start motor driver */
			xTaskCreate(
					easyStep,
					"easyStep",
					configMINIMAL_STACK_SIZE + 0,
					&easystep
					(void*) NULL,
					taskIDLE_PRIORITY + 1,
					&xEasyStepHandle);

			/* Initialize motor */
			while(HAL_GPIO_ReadPin(photointerrupter2_GPIO_Port, photointerrupter2_Pin) == 0){

			}

			// while ! init postion
				// motor step
			postion = 0;
			if(xSemaphoreTake(startPosCTRL, 100) == pdTRUE){ /* wait for start command */ 
				state = STEP;	/* go to state STEP*/
				vTask
			}
			else{
				// duurtlang
			}
		break;
		case STEP:
			// geef moter driver een schop OF MEERDERE moet ik nog nakijken
			position += stepSize;		/* keep track of the current position */
			vTaskDelay(stepSpeed);  	/* Give the motor time to move */
			state = START_MEASUREMENT; 	/* Next state START_MEASUREMENT */
		break;
		case START_MEASUREMENT:
			// geef longRangeSensorDriver een schop (positie mee geven)
			// geef shortRangeSensorDriver een schop (positie mee geven)
			// wait semaphore/signal from sensors
			// retrieve raw data from the short range sensor and store it in a register
			// retrieve raw data from the long range sensor and store it in a register
			// if all steps done
				// state =  INIT
			// if photoInterruptor //(too far)
				// state = ERROR;
			// else
				// state = STEP;
		break;
		case ERROR:
			Error_Handler();
			// report error?
			state = RESET;
		break;
		}
	}
}
#undef stepSizee
#undef stepSpeede
#undef stepDiv1e

#endif