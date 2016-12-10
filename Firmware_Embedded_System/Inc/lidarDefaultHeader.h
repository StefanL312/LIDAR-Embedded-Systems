#ifndef __DEFAULTLIDARHEADER_H
#define __DEFAULTLIDARHEADER_H

#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*To use the functions of the .c file set the define to 1*/
#define SENSDRIVE 	0
#define COMINTER 	0
#define EASYSTEP 	0
#define HBRIDGE 	0
#define PHOTOINT 	0
#define POSCTRL 	0
#define SERIALDR 	0
#define SYSCTRL 	0
#define USBDR 		0
#define EXAMPLE 	0

extern TaskHandle_t xUsbDrHandle;
extern TaskHandle_t xSerialDrHandle;
extern TaskHandle_t xComInterHandle;
extern TaskHandle_t xSensDrivHandle;
extern TaskHandle_t xPhotoIntHandle;
extern TaskHandle_t xHBridgeHandle;
extern TaskHandle_t xEasyStepHandle;
extern TaskHandle_t xSysCtrlHandle;
extern TaskHandle_t xPosCtrlHandle;

extern QueueHandle_t serialInQueue, serialOutQueue;

#endif