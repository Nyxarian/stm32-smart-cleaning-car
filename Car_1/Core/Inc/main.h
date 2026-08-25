/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_1
#define PWM2_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_2
#define TRIG_GPIO_Port GPIOA
#define CIN1_Pin GPIO_PIN_4
#define CIN1_GPIO_Port GPIOA
#define CIN2_Pin GPIO_PIN_5
#define CIN2_GPIO_Port GPIOA
#define PWM5_Pin GPIO_PIN_6
#define PWM5_GPIO_Port GPIOA
#define PWM6_Pin GPIO_PIN_7
#define PWM6_GPIO_Port GPIOA
#define VACUUM_Pin GPIO_PIN_0
#define VACUUM_GPIO_Port GPIOB
#define MPU6050_SCL_Pin GPIO_PIN_10
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_11
#define MPU6050_SDA_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_13
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define PWM7_Pin GPIO_PIN_8
#define PWM7_GPIO_Port GPIOA
#define ECHO_Pin GPIO_PIN_10
#define ECHO_GPIO_Port GPIOA
#define Clean_IN1_Pin GPIO_PIN_15
#define Clean_IN1_GPIO_Port GPIOA
#define Clean_IN2_Pin GPIO_PIN_3
#define Clean_IN2_GPIO_Port GPIOB
#define DIN1_Pin GPIO_PIN_4
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_5
#define DIN2_GPIO_Port GPIOB
#define BlueTooth_Tx_Pin GPIO_PIN_6
#define BlueTooth_Tx_GPIO_Port GPIOB
#define BlueTooth_Rx_Pin GPIO_PIN_7
#define BlueTooth_Rx_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
