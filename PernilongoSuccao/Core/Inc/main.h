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
#define QTR1_Pin GPIO_PIN_1
#define QTR1_GPIO_Port GPIOC
#define QTR2_Pin GPIO_PIN_2
#define QTR2_GPIO_Port GPIOC
#define QTR3_Pin GPIO_PIN_3
#define QTR3_GPIO_Port GPIOC
#define QTR4_Pin GPIO_PIN_0
#define QTR4_GPIO_Port GPIOA
#define QTR5_Pin GPIO_PIN_1
#define QTR5_GPIO_Port GPIOA
#define QTR6_Pin GPIO_PIN_2
#define QTR6_GPIO_Port GPIOA
#define SLESQ_Pin GPIO_PIN_3
#define SLESQ_GPIO_Port GPIOA
#define BAT_REF_Pin GPIO_PIN_1
#define BAT_REF_GPIO_Port GPIOB
#define BOTAO_Pin GPIO_PIN_10
#define BOTAO_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_11
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_13
#define STBY_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
#define PWMAVENT_Pin GPIO_PIN_8
#define PWMAVENT_GPIO_Port GPIOC
#define PWMBVENT_Pin GPIO_PIN_9
#define PWMBVENT_GPIO_Port GPIOC
#define AIN2VENT_Pin GPIO_PIN_8
#define AIN2VENT_GPIO_Port GPIOA
#define AIN1VENT_Pin GPIO_PIN_9
#define AIN1VENT_GPIO_Port GPIOA
#define STBYVENT_Pin GPIO_PIN_10
#define STBYVENT_GPIO_Port GPIOA
#define BIN1VENT_Pin GPIO_PIN_11
#define BIN1VENT_GPIO_Port GPIOA
#define BIN2VENT_Pin GPIO_PIN_12
#define BIN2VENT_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_15
#define NSS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_10
#define SCK_GPIO_Port GPIOC
#define MISO_Pin GPIO_PIN_11
#define MISO_GPIO_Port GPIOC
#define MOSI_Pin GPIO_PIN_12
#define MOSI_GPIO_Port GPIOC
#define ADIR_Pin GPIO_PIN_4
#define ADIR_GPIO_Port GPIOB
#define BDIR_Pin GPIO_PIN_5
#define BDIR_GPIO_Port GPIOB
#define AESQ_Pin GPIO_PIN_6
#define AESQ_GPIO_Port GPIOB
#define BESQ_Pin GPIO_PIN_7
#define BESQ_GPIO_Port GPIOB
#define BUZINA_Pin GPIO_PIN_9
#define BUZINA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
