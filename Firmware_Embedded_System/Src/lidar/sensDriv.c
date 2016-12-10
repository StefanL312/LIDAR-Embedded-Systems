/**
 * @file sensDriv.c
 * @author Author here
 * @date 26 nov 2016
 * @brief bla bla bla
 *
 */
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lidarDefaultHeader.h"


#if SENSDIV == 1

typedef struct
{
  uint8_t Mode;
  uint8_t Channel;
  uint8_t Take_Samples;
  uint16_t Sensor_Max_Time;
  uint16_t Sensor_Value;
  GPIO_TypeDef* GPIO;
  uint16_t GPIO_PIN
}xx_REG_InitTypeDef, *xx_REG_InitTypeDef_PTR;

#define SENSEDRIV_READY 0x01
#define SENSDRIVE_ERROR 0x02
/*enz*/

void getADCValue(xx_REG_InitTypeDef *ADC_Channel, uint16_t *Return_Value);
void CalibrateADC(void);
void ConfigureADC(uint8_t Channel);
void EnableADC(void);
void DisableADC(void);

static TaskHandle_t xSensDrivHandle;

/**
 * @brief      { This will start a measurement when it is triggered. If the measurement is completed will the value be written to Short.Sensor_Value and Long.Sensor_Value. }
 *
 * @param      pvParameters  The pvParameters	
 */
void sensDriv(void* pvParameters) 
{
	/*Get handles*/
	Short = *pvParameters[0];
	Long = *pvParameters[1];

	/* Start by calibrating the ADC sensor*/
	CalibrateADC();

	while(1) {
		/*Turn on regulators*/
		HAL_GPIO_WritePin(Long.GPIO, Long.GPIO_PIN, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(Short.GPIO, Short.GPIO_PIN, GPIO_PIN_SET);


		/*Wait for the longest delay*/
		vTaskDelay((Short.Sensor_Max_Time>=Long.Sensor_Max_Time) \
						?Short.Sensor_Max_Time \
						:Long.Sensor_Max_Time);

		/*Check for the threshold of the regulator*/
		if(HAL_GPIO_ReadPin(Long.GPIO, Long.GPIO_PIN) != GPIO_PIN_SET){
			Error_Handler();
		}

		/*Long sensor*/
		getADCValue(Long, Long.Sensor_Value);

		/*Check for the threshold of the regulator*/
		if(HAL_GPIO_ReadPin(Short.GPIO, Short.GPIO_PIN) != GPIO_PIN_SET){
			Error_Handler();
		}  

		/*Short sensor*/
		getADCValue(Short, Short.Sensor_Value);

		/*Turn off regulators*/
    	HAL_GPIO_WritePin(Long.GPIO, Long.GPIO_PIN, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(Short.GPIO, Short.GPIO_PIN, GPIO_PIN_RESET);

		/*Messages other tasks that there is a new value available*/
		xTaskNotify(xSensDrivHandle, SENSEDRIV_READY, eSetBits);

		/*Deschedule sensDriv*/
		vTaskSuspend(NULL);
	}
}

/**
 * @brief      	{ This will start a ADC measurement when it is called. If the measurement is done, it will write the ADC value to the pointer Return_Value. }
 *
 * @param      	ADC_Handle  The Handle of the ADC 	
 *				Return_Value Were to return the ADC value
 */
void getADCValue(xx_REG_InitTypeDef *ADC_Handle, uint16_t *Return_Value)
{
  uint32_t Total = 0; uint16_t Sample = 0;

  /* Configure the ADC to Channel*/
  ConfigureADC(ADC_Handle.Channel);

  /*Maximal 65536 sampels*/
  for(Sample = 0; Sample <= ADC_Handle.Take_Samples; Sample++){
  	/* critisch gebied? */
    EnableADC();
    ADC1->CR |= ADC_CR_ADSTART;

    while ((ADC1->ISR & ADC_ISR_EOC) == 0){
      /* Als het te langduurt -> Error_Handler();*/
    }
    Total += (ADC1->DR);
    DisableADC();
  	/* einde critich gebied? */
  }

  Return_Value = (Total/(ADC_Handle.Take_Samples+1));

}

/**
 * @brief      { This will start a measurement when it is triggered. If the measurement is done, it will write it to a array. }
 *
 * @param      void
 */
void CalibrateADC(void)
{
  if ((ADC1->CR & ADC_CR_ADEN) != 0){                                           /* (1) Ensure that ADEN = 0 */ 
    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);                                       /* (2) Clear ADEN*/  
  }
  ADC1->CR |= ADC_CR_ADCAL;                                                     /* (3) Launch the calibration by setting ADCAL */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0){                                       /* (4) Wait until ADCAL=0 */
    /* Als het te langduurt -> Error_Handler();*/
  }
}

/**
 * @brief      { This will start a measurement when it is triggered. If the measurement is done, it will write it to a array. }
 *
 * @param      Channel  The ADC channel to be configured.
 */
void ConfigureADC(uint8_t Channel)
{
  
  if(Channel <= 17){
    ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0 | ADC_CFGR1_EXTSEL_2;                      /* (1) Select the external trigger on TIM15_TRGO and falling edge*/
    ADC1->CHSELR = ADC_Channel[Channel];
    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;             /* (2) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us*/
    if(Channel == 16){ 
      ADC->CCR |= ADC_CCR_TSEN;
    }else if(Channel == 17){
       ADC->CCR |= ADC_CCR_VREFEN;                                              /* (3) Wake-up the Sensor (only for VBAT, Temp sensor and VRefInt)*/
    }
  }
  else{
    Error_Handler();
  }
}

/**
 * @brief      { This function enalbes the ADC. }
 *
 * @param      void
 */
void EnableADC(void)
{
  do{
    ADC1->CR |= ADC_CR_ADEN;                                                    /* (1) Enable the ADC*/
    /* Als het te langduurt -> Error_Handler();*/
  }while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)                                     /* (2) Wait until ADC ready*/;
}

/**
 * @brief      { This function disables the ADC. }
 *
 * @param      void
 */
void DisableADC(void)
{
  if ((ADC1->CR & ADC_CR_ADSTART) != 0){                                        /* (1) Ensure that no conversion on going*/
    ADC1->CR |= ADC_CR_ADSTP;                                                   /* (2) Stop any ongoing conversion*/
  }
  while ((ADC1->CR & ADC_CR_ADSTP) != 0){                                       /* (3) Wait until ADSTP is reset by hardware i.e. conversion is stopped*/
    /* Als het te langduurt -> Error_Handler();*/
  }

  ADC1->CR |= ADC_CR_ADDIS;                                                     /* (4) Disable the ADC*/
  while ((ADC1->CR & ADC_CR_ADEN) != 0){                                        /* (5) Wait until the ADC is fully disabled*/
    /* Als het te langduurt -> Error_Handler();*/
  }  
}
#endif