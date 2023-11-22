/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t run;

//int fputc(int ch,FILE *f) //printf�ض���UART
//{
//	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
//	return ch;
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	uint8_t l_line;			//���߱�־
	uint8_t r_line;			//���߱�־
//	HAL_TIM_Base_Start_IT(&htim1);
//	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	/* ��С�����״̬��ʼ�� */
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2,50);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,50);
	HAL_GPIO_WritePin(PWM1_GPIO_Port,PWM1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWM4_GPIO_Port,PWM4_Pin,GPIO_PIN_SET);
	
	
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (run==1){
			l_line=HAL_GPIO_ReadPin(IRED1_GPIO_Port,IRED1_Pin);
			r_line=HAL_GPIO_ReadPin(IRED2_GPIO_Port,IRED2_Pin);
			if (l_line==1 && r_line==0){  //��ת
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2,50);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,40);
			}
			if (l_line==0 && r_line==1){  //��ת
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2,40);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,50);
			}
			if (l_line==0 && r_line==0){  //ǰ��
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2,40);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,40);
			}
			if (l_line==1 && r_line==1){  //ֹͣ
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2,50);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,50);
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(10);

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { 
	uint8_t key;
	unsigned char uartBuf[4]="TEST";
	HAL_Delay(10);
	key=HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin);
	if(key==0) {
		if (run==0) run=1;
		else if(run==1) run=0;
	}
	key=HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin);
	if (key==0){
	HAL_UART_Transmit(&huart1,uartBuf,4,0xffff);
//		printf("Hello World");
	}
	key=HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin);
	if (key==0){


		}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
