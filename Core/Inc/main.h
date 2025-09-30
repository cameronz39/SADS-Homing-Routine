/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef *user_huart;
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIMIT_SWITCH5_Pin GPIO_PIN_13
#define LIMIT_SWITCH5_GPIO_Port GPIOC
#define LIMIT_SWITCH5_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define I2C_ADD0_Pin GPIO_PIN_5
#define I2C_ADD0_GPIO_Port GPIOA
#define I2C_ADD1_Pin GPIO_PIN_6
#define I2C_ADD1_GPIO_Port GPIOA
#define I2C_ADD2_Pin GPIO_PIN_7
#define I2C_ADD2_GPIO_Port GPIOA
#define STEP4_Pin GPIO_PIN_1
#define STEP4_GPIO_Port GPIOB
#define LIMIT_SWITCH2_Pin GPIO_PIN_2
#define LIMIT_SWITCH2_GPIO_Port GPIOB
#define LIMIT_SWITCH2_EXTI_IRQn EXTI2_IRQn
#define STEP1_Pin GPIO_PIN_13
#define STEP1_GPIO_Port GPIOB
#define STEP2_Pin GPIO_PIN_14
#define STEP2_GPIO_Port GPIOB
#define STEP3_Pin GPIO_PIN_15
#define STEP3_GPIO_Port GPIOB
#define PSEL_0_Pin GPIO_PIN_7
#define PSEL_0_GPIO_Port GPIOC
#define DIR3_Pin GPIO_PIN_8
#define DIR3_GPIO_Port GPIOC
#define DIR4_Pin GPIO_PIN_8
#define DIR4_GPIO_Port GPIOA
#define PSEL_1_Pin GPIO_PIN_9
#define PSEL_1_GPIO_Port GPIOA
#define DIR1_Pin GPIO_PIN_10
#define DIR1_GPIO_Port GPIOA
#define LIMIT_SWITCH4_Pin GPIO_PIN_11
#define LIMIT_SWITCH4_GPIO_Port GPIOA
#define LIMIT_SWITCH4_EXTI_IRQn EXTI15_10_IRQn
#define LIMIT_SWITCH3_Pin GPIO_PIN_12
#define LIMIT_SWITCH3_GPIO_Port GPIOA
#define LIMIT_SWITCH3_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DATA_READY_Pin GPIO_PIN_3
#define DATA_READY_GPIO_Port GPIOB
#define DATA_READY_EXTI_IRQn EXTI3_IRQn
#define DIR2_Pin GPIO_PIN_4
#define DIR2_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_5
#define RESET_GPIO_Port GPIOB
#define LIMIT_SWITCH1_Pin GPIO_PIN_6
#define LIMIT_SWITCH1_GPIO_Port GPIOB
#define LIMIT_SWITCH1_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
