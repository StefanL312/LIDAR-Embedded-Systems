/**
 * @file hBridge.c
 * @author Author here
 * @date 26 nov 2016
 * @brief bla bla bla
 *
 */
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lidarDefaultHeader.h"

#if HBRIDGE == 1
/**
 * @brief      Driver for the h-Bridge, Will receive 'step up' or 'step down' signals.
 *
 * @param      pvParameters  The pv parameters
 */
void hBridge(void* pvParameters) {
	while(1) {
		vTaskDelay(250);
	}
}

#endif