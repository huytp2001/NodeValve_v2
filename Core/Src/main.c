/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "addr.h"
#include "lora.h"
#include "process.h"

// TODO: 	Check rep frame build
// 				Add update routing function when timeout 
//				Fix bug toggle valve when turn off from MQTT client

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_up;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
stru_frame_t stru_frame;

/* USER CODE BEGIN PV */
static uint16_t u16_adc_value = 0;
static uint8_t au8_dma_buf[FRAME_MAX_SIZE] = {0};
static bool b_process = false;
static uint16_t u16_timer_couter = 0; // step = 0.1s
static e_frame_waiting_t e_waiting_state = WAITING_CTR;
//static bool b_waiting_ack = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void v_User_Init(void);
/* USER CODE BEGIN PFP */
void v_Mcu_Sleep(void);

// UART2 RX Callback Function
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		v_frame_set_data(au8_dma_buf, (uint8_t)(Size & 0xFF));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, au8_dma_buf, FRAME_MAX_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		
		if(e_frame_check() == FRAME_OK) b_process = true;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	u16_timer_couter++;
}

uint16_t u16_timer_get(void)
{
	return u16_timer_couter;
}

void v_timer_reset(void)
{
	u16_timer_couter = 0;
}

// Init function before enter main loop
void v_User_Init(void)
{
	HAL_GPIO_WritePin(EN_RF_GPIO_Port, EN_RF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STT_GPIO_Port, STT_Pin, GPIO_PIN_SET);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &u16_adc_value, 1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, au8_dma_buf, FRAME_MAX_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	HAL_TIM_Base_Start_IT(&htim2);
	v_addr_setup();
	v_lora_save_params(SAVE_PERMANENT, u8_lora_set_sped(PB_8N1, UDR_9600, ADR_1200), 
										 u8_lora_set_option(FM_ENABLE, IO_PUSH_PULL, WAKEUP_250, FEC_ENABLE, OPT_TP20));
	HAL_GPIO_WritePin(STT_GPIO_Port, STT_Pin, GPIO_PIN_RESET);
}

void v_Mcu_Sleep(void)
{
	HAL_SuspendTick();
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop_IT(&htim2);
	// Sleep
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	// Wakeup
	HAL_ResumeTick();
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &u16_adc_value, 1);
	HAL_TIM_Base_Start_IT(&htim2);
}

void v_uart2_transmit(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit(&huart2, pData, Size, 1000);
}

uint16_t u16_adc_get_value(void)
{
	return u16_adc_value;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	v_User_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (b_process)
		{
			v_frame_read(&stru_frame);
			// Setup next and prev addr
			v_addr_continuous_setup(stru_frame.u8_routing_val);
			switch (stru_frame.u8_frame_typeID) 
			{
				case FRAME_TYPE_CTR:
				{
					v_process_ctr_frame(&stru_frame);
					e_waiting_state = WAITING_PREV_ACK;
					b_process = false;
					
				}
				break;
				case FRAME_TYPE_REP:
				{
					v_process_rep_frame(&stru_frame);
					e_waiting_state = WAITING_NEXT_ACK;
					b_process = false;
				}
				break;
				case FRAME_TYPE_ACK_NEXT:
				{
					v_timer_reset();
					if (e_waiting_state == WAITING_NEXT_ACK) e_waiting_state = WAITING_CTR;
					if (e_waiting_state == WAITING_PREV_ACK) v_Mcu_Sleep();
					b_process = false;
				}
				break;
				case FRAME_TYPE_ACK_PREV:
				{
					v_timer_reset();
					if (e_waiting_state == WAITING_PREV_ACK) e_waiting_state = WAITING_REP;
					b_process = false;
				}
				break;
				}
			}
			switch (e_waiting_state)
			{
				case WAITING_CTR:
				{
					v_process_wait_ctr();
				}
				break;
				case WAITING_REP: 
				{
					v_process_wait_rep();
				}
				break;
				case WAITING_PREV_ACK:
				{
					if (b_is_end_node(stru_frame.u8_routing_val)) v_process_wait_ack(&stru_frame, INCREASE_DIRECTION);
					else v_process_wait_ack(&stru_frame, DECREASE_DIRECTION);
				}
				break;
				case WAITING_NEXT_ACK: 
				{
					v_process_wait_ack(&stru_frame, INCREASE_DIRECTION);
				}
				break;
				default:
				{
					v_Mcu_Sleep();
				}
				break;
			}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_M0_Pin|RF_M1_Pin|EN_RF_Pin|EN_1_Pin
                          |DIS_1_Pin|DIS_2_Pin|DIS_3_Pin|DIS_4_Pin
                          |EN_SEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_2_Pin|EN_3_Pin|EN_4_Pin|LED1_Pin
                          |LED2_Pin|LED3_Pin|LED4_Pin|STT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RF_AUX_Pin */
  GPIO_InitStruct.Pin = RF_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RF_AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_M0_Pin RF_M1_Pin EN_RF_Pin EN_1_Pin
                           DIS_1_Pin DIS_2_Pin DIS_3_Pin DIS_4_Pin
                           EN_SEN_Pin */
  GPIO_InitStruct.Pin = RF_M0_Pin|RF_M1_Pin|EN_RF_Pin|EN_1_Pin
                          |DIS_1_Pin|DIS_2_Pin|DIS_3_Pin|DIS_4_Pin
                          |EN_SEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_2_Pin EN_3_Pin EN_4_Pin LED1_Pin
                           LED2_Pin LED3_Pin LED4_Pin STT_Pin */
  GPIO_InitStruct.Pin = EN_2_Pin|EN_3_Pin|EN_4_Pin|LED1_Pin
                          |LED2_Pin|LED3_Pin|LED4_Pin|STT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW3_Pin SW4_Pin SW5_Pin
                           SW6_Pin SW7_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW3_Pin|SW4_Pin|SW5_Pin
                          |SW6_Pin|SW7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

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
