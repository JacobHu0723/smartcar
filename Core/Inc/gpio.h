/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
#ifdef      LED0_Pin
		#define LED0_Toggle()  HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin)
	  #define LED0_ON()       HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET)
	  #define LED0_OFF()      HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_RESET) 
#endif 
 
#ifdef      LED1_Pin
		#define LED1_Toggle()  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin)
	  #define LED1_ON()      HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
	  #define LED1_OFF()     HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET) 
#endif 


#ifdef      LED2_Pin
		#define LED2_Toggle() HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin)
	  #define LED2_ON()      HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET)
	  #define LED2_OFF()     HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_RESET) 
#endif 

#ifdef      LED3_Pin
		#define LED3_Toggle()  HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin)
	  #define LED3_ON()       HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET)
	  #define LED3_OFF()      HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_RESET) 
#endif 
 
#ifdef      LED4_Pin
		#define LED4_Toggle() HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin)
	  #define LED4_ON()      HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET)
	  #define LED4_OFF()     HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin, GPIO_PIN_RESET) 
#endif 


#ifdef      LED5_Pin
		#define LED5_Toggle() HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin)
	  #define LED5_ON()      HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET)
	  #define LED5_OFF()     HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin, GPIO_PIN_RESET) 
#endif 

#ifdef      LED6_Pin
		#define LED6_Toggle()  HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin)
	  #define LED6_ON()       HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_SET)
	  #define LED6_OFF()      HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin, GPIO_PIN_RESET) 
#endif 
 
#ifdef      LED7_Pin
		#define LED7_Toggle() HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin)
	  #define LED7_ON()      HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET)
	  #define LED7_OFF()     HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin, GPIO_PIN_RESET) 
#endif 




 
 
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

