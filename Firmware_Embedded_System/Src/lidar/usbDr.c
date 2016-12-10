/**
 * @file usbDr.c
 * @author Author here
 * @date 26 nov 2016
 * @brief bla bla bla
 *
 */
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lidarDefaultHeader.h"

#if USBDR == 1
/**
 * @brief      { USB handler }
 *
 * @param      pvParameters  The pv parameters
 */
void usbDr(void* pvParameters) {
	while(1) {
		vTaskDelay(250);
	}
}
#endif