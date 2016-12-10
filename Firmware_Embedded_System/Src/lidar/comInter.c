/**
 * @file comInter.c
 * @author Author here
 * @date 26 nov 2016
 * @brief bla bla bla
 *
 */
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lidarDefaultHeader.h"
 
#if COMINTER == 1
/**
 * @brief      This function Receives commands/strings from the UART interrupt. 
	This function will interpret the string.
	If a command is given, The fucntion will forward the command to the system controller.
 *
 * @param      pvParameters  The pv parameters
 */
void comInter(void* pvParameters) {
	while(1) {
		vTaskDelay(250);
	}
}

#endif