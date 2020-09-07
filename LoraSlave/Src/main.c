/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
unsigned time=0;
unsigned time_Machine_13=0;




unsigned Time_AntiR_1=0, Time_AntiR_2=0, Time_AntiR_3=0, Time_AntiR_4=0, Time_AntiR_5=0, Time_AntiR_6=0, Time_AntiR_7=0, Time_AntiR_8=0, Time_AntiR_9=0, Time_AntiR_10=0, Time_AntiR_11=0, Time_AntiR_12=0;
unsigned Time_Delay_Machine_1=0,Time_Delay_Machine_2=0,Time_Delay_Machine_3=0 ,Time_Delay_Machine_4=0,Time_Delay_Machine_5=0 ,Time_Delay_Machine_6=0,Time_Delay_Machine_7=0,Time_Delay_Machine_8=0,Time_Delay_Machine_9=0,Time_Delay_Machine_10=0,Time_Delay_Machine_11=0,Time_Delay_Machine_12=0;


/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
//Set_Param_Channel();






/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



void Set_Param_Channel(unsigned GET_ANTI_MACHINE, unsigned GET_DELAY_MACHINE, unsigned TIME_DELAY, char String_Tranmit[5])
{
	if(HAL_GetTick() - 500 > GET_ANTI_MACHINE)
	  {
		if(HAL_GetTick() - TIME_DELAY > GET_DELAY_MACHINE)
		  {
			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
			 HAL_UART_Transmit(&huart1,(uint8_t*)String_Tranmit,5,500);
			 GET_DELAY_MACHINE = HAL_GetTick();
			}
		}
GET_ANTI_MACHINE = HAL_GetTick();	
}  


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 
                           PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 
                           PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	

	switch(GPIO_Pin)
	{
		case CHANNEL_1: 
		{  
			
			if(HAL_GetTick() - 200 > Time_AntiR_1)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_1)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@S7&",5,500);
						Time_Delay_Machine_1 = HAL_GetTick();
					}
				}
				Time_AntiR_1 = HAL_GetTick();	
		}
		break;
		
		case CHANNEL_2:
		{
			if(HAL_GetTick() - 200 > Time_AntiR_2)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_2)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@E7&",5,500);
						Time_Delay_Machine_2 = HAL_GetTick();
					}
				}
				Time_AntiR_2 = HAL_GetTick();			
		}
		break;
		
		case CHANNEL_3:
		{
			if(HAL_GetTick() - 200 > Time_AntiR_3)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_3)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@S8&",5,500);
						Time_Delay_Machine_3 = HAL_GetTick();
					}
				}
				Time_AntiR_3 = HAL_GetTick();
		}
		break;
		case  CHANNEL_4: 
		{
			if(HAL_GetTick() - 200 > Time_AntiR_4)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_4)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@E8&",5,500);
						Time_Delay_Machine_4 = HAL_GetTick();
					}
				}
				Time_AntiR_4 = HAL_GetTick();
		}
		break;
	 case  CHANNEL_5: 
		{
			if(HAL_GetTick() - 200 > Time_AntiR_5)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_5)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@S9&",5,500);
						Time_Delay_Machine_5 = HAL_GetTick();
					}
				}
				Time_AntiR_5 = HAL_GetTick();		
		}
		break;
	  case  CHANNEL_6: 
		{
			if(HAL_GetTick() - 200 > Time_AntiR_6)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_6)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@E9&",5,500);
						Time_Delay_Machine_6 = HAL_GetTick();
					}
				}
				Time_AntiR_6 = HAL_GetTick();		
		}
		break;
		case  CHANNEL_7: 
		{
			if(HAL_GetTick() - 200 > Time_AntiR_7)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_7)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@S10&",5,500);
						Time_Delay_Machine_7 = HAL_GetTick();
					}
				}
				Time_AntiR_7 = HAL_GetTick();		
		}
		break;
		case  CHANNEL_8: 
		{
			if(HAL_GetTick() - 200 > Time_AntiR_8)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_8)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@E10&",5,500);
						Time_Delay_Machine_8 = HAL_GetTick();
					}
				}
				Time_AntiR_8 = HAL_GetTick();			
		}
		break;	
		case  CHANNEL_9: 
		{
			if(HAL_GetTick() - 200 > Time_AntiR_9)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_9)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@S11&",5,500);
						Time_Delay_Machine_9 = HAL_GetTick();
					}
				}
				Time_AntiR_9 = HAL_GetTick();		
		}
		break;
		case  CHANNEL_10: 
		{
			if(HAL_GetTick() - 200 > Time_AntiR_10)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_10)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@E11&",5,500);
						Time_Delay_Machine_10 = HAL_GetTick();
					}
				}
				Time_AntiR_10 = HAL_GetTick();		
		}
		break;
		case  CHANNEL_11: 
		{
			if(HAL_GetTick() - 200 > Time_AntiR_11)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_11)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@S12&",5,500);
						Time_Delay_Machine_1 = HAL_GetTick();
					}
				}
				Time_AntiR_11 = HAL_GetTick();		
		}
		break;	
		case  CHANNEL_12: 
		{
			if(HAL_GetTick() - 200 > Time_AntiR_12)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					
					if(HAL_GetTick() - 200 > Time_Delay_Machine_12)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1,(uint8_t*)"@E12&",5,500);
						Time_Delay_Machine_12 = HAL_GetTick();
					}
				}
				Time_AntiR_12 = HAL_GetTick();	
		}
		break;		
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
