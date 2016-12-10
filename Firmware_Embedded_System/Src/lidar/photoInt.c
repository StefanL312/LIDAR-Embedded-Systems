/**
 * @file photoInt.c
 * @author Author here
 * @date 26 nov 2016
 * @brief bla bla bla
 *
 */

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lidarDefaultHeader.h"

#if PHTOTINT == 1
/**
 * @brief      Debounce? not sure if we need this.. 
 *
 * @param      pvParameters  The pv parameters
 */
void photoInt(void* pvParameters) {
	while(1) {
		vTaskDelay(250);
	}
}

#endif