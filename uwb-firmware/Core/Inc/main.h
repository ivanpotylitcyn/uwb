/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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
#define IOUT_bat_Pin GPIO_PIN_1
#define IOUT_bat_GPIO_Port GPIOA
#define ACOK_bat_Pin GPIO_PIN_2
#define ACOK_bat_GPIO_Port GPIOA
#define EN_BQ_Pin GPIO_PIN_3
#define EN_BQ_GPIO_Port GPIOA
#define EN_12LED_Pin GPIO_PIN_4
#define EN_12LED_GPIO_Port GPIOA
#define EN_3V3_Pin GPIO_PIN_5
#define EN_3V3_GPIO_Port GPIOA
#define EN_6V_Pin GPIO_PIN_6
#define EN_6V_GPIO_Port GPIOA
#define EN_RS_Pin GPIO_PIN_2
#define EN_RS_GPIO_Port GPIOB
#define I2C_SCL_BME_Pin GPIO_PIN_10
#define I2C_SCL_BME_GPIO_Port GPIOB
#define I2C_SDA_BME_Pin GPIO_PIN_11
#define I2C_SDA_BME_GPIO_Port GPIOB
#define nRESET_MD_Pin GPIO_PIN_12
#define nRESET_MD_GPIO_Port GPIOB
#define nDRDY_MD_Pin GPIO_PIN_13
#define nDRDY_MD_GPIO_Port GPIOB
#define EN_5V_Pin GPIO_PIN_14
#define EN_5V_GPIO_Port GPIOB
#define UART_DE_Pin GPIO_PIN_8
#define UART_DE_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define EN_water_sens_Pin GPIO_PIN_11
#define EN_water_sens_GPIO_Port GPIOA
#define light_LED_Pin GPIO_PIN_12
#define light_LED_GPIO_Port GPIOA
#define water_sens_Pin GPIO_PIN_15
#define water_sens_GPIO_Port GPIOA
#define go_to_sleep_Pin GPIO_PIN_3
#define go_to_sleep_GPIO_Port GPIOB
#define EN_Hall_Pin GPIO_PIN_4
#define EN_Hall_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_5
#define LED_B_GPIO_Port GPIOB
#define I2C_SCL_BQ_Pin GPIO_PIN_6
#define I2C_SCL_BQ_GPIO_Port GPIOB
#define I2C_SDA_BQ_Pin GPIO_PIN_7
#define I2C_SDA_BQ_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_9
#define LED_R_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
