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
#include "stm32h7xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UART_TX_Pin GPIO_PIN_10
#define UART_TX_GPIO_Port GPIOC
#define Digital_OUT_0_Pin GPIO_PIN_1
#define Digital_OUT_0_GPIO_Port GPIOE
#define UART_RX_Pin GPIO_PIN_11
#define UART_RX_GPIO_Port GPIOC
#define Digital_OUT_6_Pin GPIO_PIN_2
#define Digital_OUT_6_GPIO_Port GPIOE
#define Digital_OUT_1_Pin GPIO_PIN_0
#define Digital_OUT_1_GPIO_Port GPIOE
#define UART_TX_EN_Pin GPIO_PIN_12
#define UART_TX_EN_GPIO_Port GPIOC
#define Digital_OUT_5_Pin GPIO_PIN_5
#define Digital_OUT_5_GPIO_Port GPIOE
#define Digital_OUT_4_Pin GPIO_PIN_4
#define Digital_OUT_4_GPIO_Port GPIOE
#define Digital_OUT_3_Pin GPIO_PIN_3
#define Digital_OUT_3_GPIO_Port GPIOE
#define RS485_EN_120R_Pin GPIO_PIN_10
#define RS485_EN_120R_GPIO_Port GPIOA
#define Digital_OUT_2_Pin GPIO_PIN_6
#define Digital_OUT_2_GPIO_Port GPIOE
#define RS485_SLR_Control_Pin GPIO_PIN_8
#define RS485_SLR_Control_GPIO_Port GPIOC
#define DO_V_A0_Pin GPIO_PIN_1
#define DO_V_A0_GPIO_Port GPIOF
#define DO_BUF_EN_N_Pin GPIO_PIN_4
#define DO_BUF_EN_N_GPIO_Port GPIOF
#define DI_V_A0_Pin GPIO_PIN_8
#define DI_V_A0_GPIO_Port GPIOF
#define Digital_IN_0_Pin GPIO_PIN_10
#define Digital_IN_0_GPIO_Port GPIOE
#define DI_BUF_EN_N_Pin GPIO_PIN_14
#define DI_BUF_EN_N_GPIO_Port GPIOF
#define Digital_IN_1_Pin GPIO_PIN_9
#define Digital_IN_1_GPIO_Port GPIOE
#define Digital_IN_2_Pin GPIO_PIN_11
#define Digital_IN_2_GPIO_Port GPIOE
#define Digital_IN_3_Pin GPIO_PIN_12
#define Digital_IN_3_GPIO_Port GPIOE
#define Digital_IN_4_Pin GPIO_PIN_15
#define Digital_IN_4_GPIO_Port GPIOE
#define Digital_IN_7_Pin GPIO_PIN_8
#define Digital_IN_7_GPIO_Port GPIOE
#define Digital_IN_5_Pin GPIO_PIN_13
#define Digital_IN_5_GPIO_Port GPIOE
#define Digital_OUT_7_Pin GPIO_PIN_7
#define Digital_OUT_7_GPIO_Port GPIOE
#define Digital_IN_6_Pin GPIO_PIN_14
#define Digital_IN_6_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
