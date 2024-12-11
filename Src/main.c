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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#include "fanatec.h"

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

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

GPIO_PinState pins_debounced[NUMBER_OF_VIRTUAL_INPUTS] = {0};

struct encoder_t encoders[NUMBER_OF_ENCODERS];

uint32_t ADC_DMA_buffer[2] = {0};
uint8_t new_ADC_data = 0;

struct fanatec_data_out_t fanatec_data_out;

char received_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void Pin_To_Bit(uint8_t* bitfield, uint8_t bit_index, GPIO_PinState pin_state);
uint8_t Compute_Checksum(const uint8_t* buffer, uint8_t length);

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

  for (int i = 0; i < NUMBER_OF_ENCODERS; i++)
  {
    encoders[i].state      = Idle;
    encoders[i].A          = GPIO_PIN_SET;
    encoders[i].B          = GPIO_PIN_SET;
    encoders[i].previous_A = GPIO_PIN_SET;
    encoders[i].previous_B = GPIO_PIN_SET;
  }

  memset(fanatec_data_out.raw, 0, sizeof (fanatec_data_out.raw));

  fanatec_data_out.header = 0xA5;
  fanatec_data_out.id     = 0x04; // ID for universal HUB
  fanatec_data_out.fwvers = 0x13; // firmware version for universal HUB
  fanatec_data_out.crc    = Compute_CRC(fanatec_data_out.raw, sizeof(fanatec_data_out.raw) - 1);

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, ADC_DMA_buffer, 2);

  HAL_TIM_Base_Start_IT(&htim1);

  uint8_t UART_buffer[7] = {0};
  UART_buffer[0] = 0xA5;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Map inputs to UART buffer
    Pin_To_Bit(&UART_buffer[1], 0, pins_debounced[0]);  // Right shifter paddle
    Pin_To_Bit(&UART_buffer[1], 1, pins_debounced[1]);  // Left shifter paddle
    Pin_To_Bit(&UART_buffer[1], 2, pins_debounced[2]);  // Right back button
    Pin_To_Bit(&UART_buffer[1], 3, pins_debounced[3]);  // Left back button
    Pin_To_Bit(&UART_buffer[1], 4, pins_debounced[4]);  // D-pad up
    Pin_To_Bit(&UART_buffer[1], 5, pins_debounced[5]);  // D-pad down
    Pin_To_Bit(&UART_buffer[1], 6, pins_debounced[6]);  // D-pad right
    Pin_To_Bit(&UART_buffer[1], 7, pins_debounced[7]);  // D-pad left
    if (pins_debounced[4] == GPIO_PIN_SET &&
        pins_debounced[5] == GPIO_PIN_SET &&
        pins_debounced[6] == GPIO_PIN_SET &&
        pins_debounced[7] == GPIO_PIN_SET)
      Pin_To_Bit(&UART_buffer[2], 0, pins_debounced[8]); // D-pad button
    else
      Pin_To_Bit(&UART_buffer[2], 0, GPIO_PIN_SET); // D-pad button
    Pin_To_Bit(&UART_buffer[2], 1, pins_debounced[21]); // Right 1
    Pin_To_Bit(&UART_buffer[2], 2, pins_debounced[22]); // Right 2
    Pin_To_Bit(&UART_buffer[2], 3, pins_debounced[23]); // Right 3
    Pin_To_Bit(&UART_buffer[2], 4, pins_debounced[24]); // Right 4
    Pin_To_Bit(&UART_buffer[2], 5, pins_debounced[25]); // Right 5
    Pin_To_Bit(&UART_buffer[2], 6, pins_debounced[26]); // Right 6
    Pin_To_Bit(&UART_buffer[2], 7, pins_debounced[27]); // Left 1
    Pin_To_Bit(&UART_buffer[3], 0, pins_debounced[28]); // Left 2
    Pin_To_Bit(&UART_buffer[3], 1, pins_debounced[29]); // Left 3
    Pin_To_Bit(&UART_buffer[3], 2, pins_debounced[30]); // Left 4
    Pin_To_Bit(&UART_buffer[3], 3, pins_debounced[31]); // Left 5
    Pin_To_Bit(&UART_buffer[3], 4, pins_debounced[32]); // Left 6
    Pin_To_Bit(&UART_buffer[3], 5, (encoders[0].state == Clockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[3], 6, (encoders[0].state == Counterclockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[3], 7, (encoders[1].state == Clockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[4], 0, (encoders[1].state == Counterclockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[4], 1, (encoders[2].state == Clockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[4], 2, (encoders[2].state == Counterclockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[4], 3, (encoders[3].state == Clockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[4], 4, (encoders[3].state == Counterclockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[4], 5, (encoders[4].state == Clockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[4], 6, (encoders[4].state == Counterclockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[4], 7, (encoders[5].state == Clockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Pin_To_Bit(&UART_buffer[5], 0, (encoders[5].state == Counterclockwise) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    UART_buffer[6] = Compute_Checksum(UART_buffer, sizeof(UART_buffer) - 1);

    HAL_UART_Transmit(&huart1, UART_buffer, sizeof(UART_buffer), HAL_MAX_DELAY);

    HAL_Delay (2);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 95;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DPAD_ENCODER_A_Pin DPAD_ENCODER_B_Pin */
  GPIO_InitStruct.Pin = DPAD_ENCODER_A_Pin|DPAD_ENCODER_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHIFTER_PADDLE_RIGHT_Pin SHIFTER_PADDLE_LEFT_Pin DPAD_BUTTON_Pin BACK_BUTTON_RIGHT_Pin
                           BACK_BUTTON_LEFT_Pin */
  GPIO_InitStruct.Pin = SHIFTER_PADDLE_RIGHT_Pin|SHIFTER_PADDLE_LEFT_Pin|DPAD_BUTTON_Pin|BACK_BUTTON_RIGHT_Pin
                          |BACK_BUTTON_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_1_A_Pin ENCODER_1_B_Pin ENCODER_2_B_Pin DPAD_UP_Pin
                           DPAD_DOWN_Pin DPAD_RIGHT_Pin DPAD_LEFT_Pin ENCODER_3_A_Pin
                           ENCODER_3_B_Pin ENCODER_4_A_Pin ENCODER_4_B_Pin ENCODER_5_A_Pin
                           ENCODER_5_B_Pin ENCODER_2_A_Pin */
  GPIO_InitStruct.Pin = ENCODER_1_A_Pin|ENCODER_1_B_Pin|ENCODER_2_B_Pin|DPAD_UP_Pin
                          |DPAD_DOWN_Pin|DPAD_RIGHT_Pin|DPAD_LEFT_Pin|ENCODER_3_A_Pin
                          |ENCODER_3_B_Pin|ENCODER_4_A_Pin|ENCODER_4_B_Pin|ENCODER_5_A_Pin
                          |ENCODER_5_B_Pin|ENCODER_2_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    new_ADC_data = 1;
}

void SPI1_CS_High_IRQ_Handler(void)
{
  HAL_SPI_Transmit_DMA(&hspi1, fanatec_data_out.raw, sizeof(fanatec_data_out.raw));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  static GPIO_TypeDef* const GPIO_port[NUMBER_OF_DIGITAL_INPUTS] =
   {SHIFTER_PADDLE_RIGHT_GPIO_Port,
    SHIFTER_PADDLE_LEFT_GPIO_Port,
    BACK_BUTTON_RIGHT_GPIO_Port,
    BACK_BUTTON_LEFT_GPIO_Port,
    DPAD_UP_GPIO_Port,
    DPAD_DOWN_GPIO_Port,
    DPAD_RIGHT_GPIO_Port,
    DPAD_LEFT_GPIO_Port,
    DPAD_BUTTON_GPIO_Port,
    DPAD_ENCODER_A_GPIO_Port,
    DPAD_ENCODER_B_GPIO_Port,
    ENCODER_1_A_GPIO_Port,
    ENCODER_1_B_GPIO_Port,
    ENCODER_2_A_GPIO_Port,
    ENCODER_2_B_GPIO_Port,
    ENCODER_3_A_GPIO_Port,
    ENCODER_3_B_GPIO_Port,
    ENCODER_4_A_GPIO_Port,
    ENCODER_4_B_GPIO_Port,
    ENCODER_5_A_GPIO_Port,
    ENCODER_5_B_GPIO_Port};

  static const uint16_t GPIO_pin[NUMBER_OF_DIGITAL_INPUTS] =
   {SHIFTER_PADDLE_RIGHT_Pin,
    SHIFTER_PADDLE_LEFT_Pin,
    BACK_BUTTON_RIGHT_Pin,
    BACK_BUTTON_LEFT_Pin,
    DPAD_UP_Pin,
    DPAD_DOWN_Pin,
    DPAD_RIGHT_Pin,
    DPAD_LEFT_Pin,
    DPAD_BUTTON_Pin,
    DPAD_ENCODER_A_Pin,
    DPAD_ENCODER_B_Pin,
    ENCODER_1_A_Pin,
    ENCODER_1_B_Pin,
    ENCODER_2_A_Pin,
    ENCODER_2_B_Pin,
    ENCODER_3_A_Pin,
    ENCODER_3_B_Pin,
    ENCODER_4_A_Pin,
    ENCODER_4_B_Pin,
    ENCODER_5_A_Pin,
    ENCODER_5_B_Pin};

  static GPIO_PinState pins_state[NUMBER_OF_VIRTUAL_INPUTS] = {0};
  static GPIO_PinState previous_pins_state[NUMBER_OF_VIRTUAL_INPUTS] = {0};

  static uint8_t nb_cycles_stable[NUMBER_OF_VIRTUAL_INPUTS] = {0};

  // Read digital inputs
  for (int i = 0; i < NUMBER_OF_DIGITAL_INPUTS; i++)
  {
    pins_state[i] = HAL_GPIO_ReadPin(GPIO_port[i], GPIO_pin[i]);
  }

  // Compute buttons connected to ADC state
  if (new_ADC_data == 1)
  {
    new_ADC_data = 0;

    // Reset all pins first (active low)
    for (int i = 0; i < NUMBER_OF_ADC_BUTTONS; i++)
    {
      pins_state[NUMBER_OF_DIGITAL_INPUTS + i] = GPIO_PIN_SET;
    }

    for (int i = 0; i < 2; i++)
    {
      if (ADC_DMA_buffer[i] < 21)
        pins_state[NUMBER_OF_DIGITAL_INPUTS + (i * 6)] = GPIO_PIN_RESET;
      else if (ADC_DMA_buffer[i] < 64)
        pins_state[NUMBER_OF_DIGITAL_INPUTS + (i * 6) + 1] = GPIO_PIN_RESET;
      else if (ADC_DMA_buffer[i] < 106)
        pins_state[NUMBER_OF_DIGITAL_INPUTS + (i * 6) + 2] = GPIO_PIN_RESET;
      else if (ADC_DMA_buffer[i] < 149)
        pins_state[NUMBER_OF_DIGITAL_INPUTS + (i * 6) + 3] = GPIO_PIN_RESET;
      else if (ADC_DMA_buffer[i] < 191)
        pins_state[NUMBER_OF_DIGITAL_INPUTS + (i * 6) + 4] = GPIO_PIN_RESET;
      else if (ADC_DMA_buffer[i] < 234)
        pins_state[NUMBER_OF_DIGITAL_INPUTS + (i * 6) + 5] = GPIO_PIN_RESET;
    }

    HAL_ADC_Start_DMA(&hadc1, ADC_DMA_buffer, 2);
  }

  // Debounce inputs
  for (int i = 0; i < NUMBER_OF_VIRTUAL_INPUTS; i++)
  {
    if (previous_pins_state[i] == pins_state[i])
    {
      nb_cycles_stable[i]++;
    }
    else
    {
      previous_pins_state[i] = pins_state[i];
      nb_cycles_stable[i]    = 0;
    }

    if (nb_cycles_stable[i] >= DEBOUNCE_CYCLES)
    {
      pins_debounced[i] = pins_state[i];
    }
  }

  // Compute encoders state
  for (int i = 0; i < NUMBER_OF_ENCODERS; i++)
  {
    uint8_t A_index = NUMBER_OF_BUTTONS + (i * 2);
    uint8_t B_index = NUMBER_OF_BUTTONS + (i * 2) + 1;

    encoders[i].A = pins_debounced[A_index];
    encoders[i].B = pins_debounced[B_index];

    switch (encoders[i].state)
    {
      case Idle:
        if (encoders[i].A == GPIO_PIN_RESET
            && encoders[i].A != encoders[i].previous_A
            && encoders[i].B == GPIO_PIN_SET)
        {
          encoders[i].state = Clockwise;
        }
        else if (encoders[i].B == GPIO_PIN_RESET
            && encoders[i].B != encoders[i].previous_B
            && encoders[i].A == GPIO_PIN_SET)
        {
          encoders[i].state = Counterclockwise;
        }
        break;

      case Clockwise:
          if (encoders[i].A == GPIO_PIN_SET)
          {
            encoders[i].state = Idle;
          }
        break;

      case Counterclockwise:
          if (encoders[i].B == GPIO_PIN_SET)
          {
            encoders[i].state = Idle;
          }
        break;
    }

    encoders[i].previous_A = encoders[i].A;
    encoders[i].previous_B = encoders[i].B;
  }

  // Heartbeat LED at 1Hz
  static uint16_t LED_counter = 0;

  if (LED_counter > 500)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    LED_counter = 0;
  }
  else
  {
    LED_counter++;
  }
}

void Pin_To_Bit(uint8_t* bitfield, uint8_t bit_index, GPIO_PinState pin_state)
{
  if (pin_state == GPIO_PIN_SET)
  {
    *bitfield &= ~(1 << bit_index);
  }
  else
  {
    *bitfield |= (1 << bit_index);
  }
}

uint8_t Compute_Checksum(const uint8_t* buffer, uint8_t length)
{
  uint8_t checksum = 0;

  for (int i = 0; i < length; i++)
  {
    checksum ^= buffer[i];
  }

  return checksum;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
