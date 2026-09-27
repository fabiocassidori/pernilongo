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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SLDIR_Pin GPIO_PIN_0
#define SLDIR_GPIO_Port GPIOC
#define QTR12_Pin GPIO_PIN_3
#define QTR12_GPIO_Port GPIOC
#define QTR11_Pin GPIO_PIN_0
#define QTR11_GPIO_Port GPIOA
#define QTR10_Pin GPIO_PIN_1
#define QTR10_GPIO_Port GPIOA
#define QTR09_Pin GPIO_PIN_2
#define QTR09_GPIO_Port GPIOA
#define QTR08_Pin GPIO_PIN_3
#define QTR08_GPIO_Port GPIOA
#define QTR07_Pin GPIO_PIN_4
#define QTR07_GPIO_Port GPIOA
#define QTR06_Pin GPIO_PIN_5
#define QTR06_GPIO_Port GPIOA
#define QTR05_Pin GPIO_PIN_6
#define QTR05_GPIO_Port GPIOA
#define QTR04_Pin GPIO_PIN_7
#define QTR04_GPIO_Port GPIOA
#define QTR03_Pin GPIO_PIN_4
#define QTR03_GPIO_Port GPIOC
#define QTR02_Pin GPIO_PIN_5
#define QTR02_GPIO_Port GPIOC
#define QTR01_Pin GPIO_PIN_0
#define QTR01_GPIO_Port GPIOB
#define SLESQ_Pin GPIO_PIN_1
#define SLESQ_GPIO_Port GPIOB
#define DIRA_Pin GPIO_PIN_12
#define DIRA_GPIO_Port GPIOB
#define DIS_Pin GPIO_PIN_13
#define DIS_GPIO_Port GPIOB
#define DIRB_Pin GPIO_PIN_14
#define DIRB_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOC
#define PWM_SENSOR_Pin GPIO_PIN_8
#define PWM_SENSOR_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_9
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_10
#define BT_RX_GPIO_Port GPIOA
#define PWM_VACUO_Pin GPIO_PIN_15
#define PWM_VACUO_GPIO_Port GPIOA
#define BOTAO_Pin GPIO_PIN_4
#define BOTAO_GPIO_Port GPIOB
#define BUZINA_Pin GPIO_PIN_8
#define BUZINA_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_9
#define LED_R_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
