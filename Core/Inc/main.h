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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union
{
	uint8_t buf[ 7 ];
	struct{
		uint8_t Year;
		uint8_t Month;
		uint8_t Date;
		uint8_t WeekDay;
		uint8_t Hours;
		uint8_t Minutes;
		uint8_t Seconds;
	}map;
}calendar_t;

typedef struct
{
  uint8_t num1;
} Data_t;

typedef struct {
	uint8_t Status;
	uint8_t Number;
}Touch_t;
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
void set_clock_to_RTC ( calendar_t calendar );
void get_clock_from_RTC ( calendar_t *calendar );
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_BL_CTRL_Pin GPIO_PIN_7
#define LCD_BL_CTRL_GPIO_Port GPIOF
#define SD_CD_Pin GPIO_PIN_4
#define SD_CD_GPIO_Port GPIOH
#define PWM_BL_Pin GPIO_PIN_0
#define PWM_BL_GPIO_Port GPIOB
#define GPIO_OUT_BEEP_Pin GPIO_PIN_5
#define GPIO_OUT_BEEP_GPIO_Port GPIOD
#define CTP_INT_Pin GPIO_PIN_3
#define CTP_INT_GPIO_Port GPIOB
#define CTP_INT_EXTI_IRQn EXTI3_IRQn
#define CTP_RST_Pin GPIO_PIN_4
#define CTP_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
