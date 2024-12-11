/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void SPI1_CS_High_IRQ_Handler(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define DPAD_ENCODER_A_Pin GPIO_PIN_14
#define DPAD_ENCODER_A_GPIO_Port GPIOC
#define DPAD_ENCODER_B_Pin GPIO_PIN_15
#define DPAD_ENCODER_B_GPIO_Port GPIOC
#define ADC_BUTTONS_RIGHT_Pin GPIO_PIN_0
#define ADC_BUTTONS_RIGHT_GPIO_Port GPIOA
#define ADC_BUTTONS_LEFT_Pin GPIO_PIN_1
#define ADC_BUTTONS_LEFT_GPIO_Port GPIOA
#define SHIFTER_PADDLE_RIGHT_Pin GPIO_PIN_2
#define SHIFTER_PADDLE_RIGHT_GPIO_Port GPIOA
#define SHIFTER_PADDLE_LEFT_Pin GPIO_PIN_3
#define SHIFTER_PADDLE_LEFT_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI1_CS_EXTI_IRQn EXTI4_IRQn
#define ENCODER_1_A_Pin GPIO_PIN_0
#define ENCODER_1_A_GPIO_Port GPIOB
#define ENCODER_1_B_Pin GPIO_PIN_1
#define ENCODER_1_B_GPIO_Port GPIOB
#define ENCODER_2_B_Pin GPIO_PIN_10
#define ENCODER_2_B_GPIO_Port GPIOB
#define DPAD_UP_Pin GPIO_PIN_12
#define DPAD_UP_GPIO_Port GPIOB
#define DPAD_DOWN_Pin GPIO_PIN_13
#define DPAD_DOWN_GPIO_Port GPIOB
#define DPAD_RIGHT_Pin GPIO_PIN_14
#define DPAD_RIGHT_GPIO_Port GPIOB
#define DPAD_LEFT_Pin GPIO_PIN_15
#define DPAD_LEFT_GPIO_Port GPIOB
#define DPAD_BUTTON_Pin GPIO_PIN_8
#define DPAD_BUTTON_GPIO_Port GPIOA
#define BACK_BUTTON_RIGHT_Pin GPIO_PIN_11
#define BACK_BUTTON_RIGHT_GPIO_Port GPIOA
#define BACK_BUTTON_LEFT_Pin GPIO_PIN_12
#define BACK_BUTTON_LEFT_GPIO_Port GPIOA
#define ENCODER_3_A_Pin GPIO_PIN_3
#define ENCODER_3_A_GPIO_Port GPIOB
#define ENCODER_3_B_Pin GPIO_PIN_4
#define ENCODER_3_B_GPIO_Port GPIOB
#define ENCODER_4_A_Pin GPIO_PIN_5
#define ENCODER_4_A_GPIO_Port GPIOB
#define ENCODER_4_B_Pin GPIO_PIN_6
#define ENCODER_4_B_GPIO_Port GPIOB
#define ENCODER_5_A_Pin GPIO_PIN_7
#define ENCODER_5_A_GPIO_Port GPIOB
#define ENCODER_5_B_Pin GPIO_PIN_8
#define ENCODER_5_B_GPIO_Port GPIOB
#define ENCODER_2_A_Pin GPIO_PIN_9
#define ENCODER_2_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define NUMBER_OF_BUTTONS     9
#define NUMBER_OF_ENCODERS    6
#define NUMBER_OF_ADC_BUTTONS 12

#define NUMBER_OF_DIGITAL_INPUTS NUMBER_OF_BUTTONS + (NUMBER_OF_ENCODERS * 2)
#define NUMBER_OF_VIRTUAL_INPUTS NUMBER_OF_ADC_BUTTONS + NUMBER_OF_DIGITAL_INPUTS

#define DEBOUNCE_CYCLES 10

enum encoder_state_t
{
  Idle = 0,
  Clockwise,
  Counterclockwise
};

struct encoder_t
{
  enum encoder_state_t state;
  GPIO_PinState A;
  GPIO_PinState B;
  GPIO_PinState previous_A;
  GPIO_PinState previous_B;
};


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
