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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

struct fanatec_data_out_t fanatec_data_out;

char received_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

int _write(int fd, char* ptr, int len);

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  memset(fanatec_data_out.raw, 0, sizeof (fanatec_data_out.raw));

  fanatec_data_out.header = 0xA5;
  fanatec_data_out.id     = 0x04; // ID for universal HUB
  fanatec_data_out.fwvers = 0x13; // firmware version for universal HUB
  fanatec_data_out.crc    = Compute_CRC(fanatec_data_out.raw, sizeof(fanatec_data_out.raw) - 1);

  printf("Start main loop\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HAL_UART_Receive(&huart1, (uint8_t *)&received_data, 1, HAL_MAX_DELAY) == HAL_OK)
    {
      printf("Received: %c\n", received_data);
    }

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

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
  huart1.Init.BaudRate = 921600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SPI1_CS_High_IRQ_Handler(void)
{
  switch (received_data)
  {
    case 'a':
      fanatec_data_out.buttons[0] = 0x01;
      break;

    case 'b':
      fanatec_data_out.buttons[0] = 0x02;
      break;

    case 'c':
      fanatec_data_out.buttons[0] = 0x04;
      break;

    case 'd':
      fanatec_data_out.buttons[0] = 0x08;
      break;

    case 'e':
      fanatec_data_out.buttons[0] = 0x10;
      break;

    case 'f':
      fanatec_data_out.buttons[0] = 0x20;
      break;

    case 'g':
      fanatec_data_out.buttons[0] = 0x40;
      break;

    case 'h':
      fanatec_data_out.buttons[0] = 0x80;
      break;

    case 'i':
      fanatec_data_out.buttons[1] = 0x01;
      break;

    case 'j':
      fanatec_data_out.buttons[1] = 0x02;
      break;

    case 'k':
      fanatec_data_out.buttons[1] = 0x04;
      break;

    case 'l':
      fanatec_data_out.buttons[1] = 0x08;
      break;

    case 'm':
      fanatec_data_out.buttons[1] = 0x10;
      break;

    case 'n':
      fanatec_data_out.buttons[1] = 0x20;
      break;

    case 'o':
      fanatec_data_out.buttons[1] = 0x40;
      break;

    case 'p':
      fanatec_data_out.buttons[1] = 0x80;
      break;

    case 'q':
      fanatec_data_out.buttons[2] = 0x01;
      break;

    case 'r':
      fanatec_data_out.buttons[2] = 0x02;
      break;

    case 's':
      fanatec_data_out.buttons[2] = 0x04;
      break;

    case 't':
      fanatec_data_out.buttons[2] = 0x08;
      break;

    case 'u':
      fanatec_data_out.buttons[2] = 0x10;
      break;

    case 'v':
      fanatec_data_out.buttons[2] = 0x20;
      break;

    case 'w':
      fanatec_data_out.buttons[2] = 0x40;
      break;

    case 'x':
      fanatec_data_out.buttons[2] = 0x80;
      break;

    case 'y':
      fanatec_data_out.encoder = 1;
      break;

    case 'z':
      fanatec_data_out.encoder = -1;
      break;

    case 'A':
      fanatec_data_out.btnHub[0] = 0x01;
      break;

    case 'B':
      fanatec_data_out.btnHub[0] = 0x02;
      break;

    case 'C':
      fanatec_data_out.btnHub[0] = 0x04;
      break;

    case 'D':
      fanatec_data_out.btnHub[0] = 0x08;
      break;

    case 'E':
      fanatec_data_out.btnHub[0] = 0x10;
      break;

    case 'F':
      fanatec_data_out.btnHub[0] = 0x20;
      break;

    case 'G':
      fanatec_data_out.btnHub[0] = 0x40;
      break;

    case 'H':
      fanatec_data_out.btnHub[0] = 0x80;
      break;

    case 'I':
      fanatec_data_out.btnHub[1] = 0x01;
      break;

    case 'J':
      fanatec_data_out.btnHub[1] = 0x02;
      break;

    case 'K':
      fanatec_data_out.btnHub[1] = 0x04;
      break;

    case 'L':
      fanatec_data_out.btnHub[1] = 0x08;
      break;

    case 'M':
      fanatec_data_out.btnHub[1] = 0x10;
      break;

    case 'N':
      fanatec_data_out.btnHub[1] = 0x20;
      break;

    case 'O':
      fanatec_data_out.btnHub[1] = 0x40;
      break;

    case 'P':
      fanatec_data_out.btnHub[1] = 0x80;
      break;

    case 'Q':
      fanatec_data_out.btnPS[0] = 0x01;
      break;

    case 'R':
      fanatec_data_out.btnPS[0] = 0x02;
      break;

    case 'S':
      fanatec_data_out.btnPS[0] = 0x04;
      break;

    case 'T':
      fanatec_data_out.btnPS[0] = 0x08;
      break;

    case 'U':
      fanatec_data_out.btnPS[0] = 0x10;
      break;

    case 'V':
      fanatec_data_out.btnPS[0] = 0x20;
      break;

    case 'W':
      fanatec_data_out.btnPS[0] = 0x40;
      break;

    case 'X':
      fanatec_data_out.btnPS[0] = 0x80;
      break;

    case 'Y':
      fanatec_data_out.btnPS[1] = 0x01;
      break;

    case 'Z':
      fanatec_data_out.btnPS[1] = 0x02;
      break;

    case '0':
      fanatec_data_out.btnPS[1] = 0x04;
      break;

    case '1':
      fanatec_data_out.btnPS[1] = 0x08;
      break;

    case '2':
      fanatec_data_out.btnPS[1] = 0x10;
      break;

    case '3':
      fanatec_data_out.btnPS[1] = 0x20;
      break;

    case '4':
      fanatec_data_out.btnPS[1] = 0x40;
      break;

    case '5':
      fanatec_data_out.btnPS[1] = 0x80;
      break;

    case '6':
      fanatec_data_out.axisX = 127;
      break;

    case '7':
      fanatec_data_out.axisY = 127;
      break;

    default:
      fanatec_data_out.buttons[0] = 0;
      fanatec_data_out.buttons[1] = 0;
      fanatec_data_out.buttons[2] = 0;
      fanatec_data_out.encoder = 0;
      fanatec_data_out.btnHub[0] = 0;
      fanatec_data_out.btnHub[1] = 0;
      fanatec_data_out.btnPS[0] = 0;
      fanatec_data_out.btnPS[1] = 0;
      fanatec_data_out.axisX = -128;
      fanatec_data_out.axisY = -128;
  }

  fanatec_data_out.crc = Compute_CRC(fanatec_data_out.raw, sizeof(fanatec_data_out.raw) - 1);

  if (HAL_SPI_Transmit_DMA(&hspi1, fanatec_data_out.raw, sizeof(fanatec_data_out.raw)) != HAL_OK)
  {
    Error_Handler();
  }
}

int _write(int fd, char* ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
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

  printf("Error_Handler\n");

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
