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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/******SERIAL COMMUNICATION SETTINGS******/
#define USART2_RX_BUFFER_SIZE  128
extern uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
extern volatile uint16_t usart2_prev_index;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void UART_IdleProcess(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t buffer_size);
void ProcessData(uint8_t* data, uint16_t len, UART_HandleTypeDef *huart);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Thermistor6_Pin GPIO_PIN_0
#define Thermistor6_GPIO_Port GPIOC
#define Thermistor5_Pin GPIO_PIN_1
#define Thermistor5_GPIO_Port GPIOC
#define Thermistor8_Pin GPIO_PIN_2
#define Thermistor8_GPIO_Port GPIOC
#define Thermistor7_Pin GPIO_PIN_3
#define Thermistor7_GPIO_Port GPIOC
#define Thermistor1_Pin GPIO_PIN_0
#define Thermistor1_GPIO_Port GPIOA
#define Thermistor2_Pin GPIO_PIN_1
#define Thermistor2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Thermistor3_Pin GPIO_PIN_4
#define Thermistor3_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_6
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_7
#define PWM4_GPIO_Port GPIOA
#define Thermistor9_Pin GPIO_PIN_4
#define Thermistor9_GPIO_Port GPIOC
#define Thermistor4_Pin GPIO_PIN_0
#define Thermistor4_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_1
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_8
#define PWM2_GPIO_Port GPIOC
#define PWM8_Pin GPIO_PIN_8
#define PWM8_GPIO_Port GPIOA
#define PWM7_Pin GPIO_PIN_9
#define PWM7_GPIO_Port GPIOA
#define PWM9_Pin GPIO_PIN_10
#define PWM9_GPIO_Port GPIOA
#define PWM6_Pin GPIO_PIN_11
#define PWM6_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define PWM5_Pin GPIO_PIN_6
#define PWM5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
