/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define sensor_short_Pin GPIO_PIN_0
#define sensor_short_GPIO_Port GPIOA
#define sensor_long_Pin GPIO_PIN_1
#define sensor_long_GPIO_Port GPIOA
#define VCC_Pin GPIO_PIN_2
#define VCC_GPIO_Port GPIOA
#define n_u_Pin GPIO_PIN_3
#define n_u_GPIO_Port GPIOA
#define button3_Pin GPIO_PIN_4
#define button3_GPIO_Port GPIOA
#define button2_Pin GPIO_PIN_5
#define button2_GPIO_Port GPIOA
#define button1_Pin GPIO_PIN_6
#define button1_GPIO_Port GPIOA
#define status_led3_Pin GPIO_PIN_0
#define status_led3_GPIO_Port GPIOB
#define status_led2_Pin GPIO_PIN_1
#define status_led2_GPIO_Port GPIOB
#define status_led1_Pin GPIO_PIN_2
#define status_led1_GPIO_Port GPIOB
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
#define psu_1_reset_Pin GPIO_PIN_15
#define psu_1_reset_GPIO_Port GPIOA
#define psu_2_reset_Pin GPIO_PIN_4
#define psu_2_reset_GPIO_Port GPIOB
#define psu_2_enable_Pin GPIO_PIN_5
#define psu_2_enable_GPIO_Port GPIOB
#define psu_3_reset_Pin GPIO_PIN_6
#define psu_3_reset_GPIO_Port GPIOB
#define psu_3_enable_Pin GPIO_PIN_7
#define psu_3_enable_GPIO_Port GPIOB
#define photointerrupter1_Pin GPIO_PIN_8
#define photointerrupter1_GPIO_Port GPIOB
#define photointerrupter2_Pin GPIO_PIN_9
#define photointerrupter2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
