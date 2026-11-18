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

/* USER CODE BEGIN Private defines */
// Alamat I2C Sensor (format 8-bit untuk fungsi HAL_I2C)
#define MPU6050_I2C_ADDR      (0x68 << 1)
#define BARO_I2C_ADDR         (0x76 << 1) // Ubah ke (0x77 << 1) jika pin SDO barometer ke 3.3V

// Pemetaan Kanal PWM 3 Servo pada TIM3 (PA6, PA7, PB0)
#define SERVO1_CHANNEL   TIM_CHANNEL_1 // PA6 (Aileron kiri)
#define SERVO2_CHANNEL   TIM_CHANNEL_2 // PA7 (Elevator)
#define SERVO3_CHANNEL   TIM_CHANNEL_3 // PB0 (Rudder)
#define SERVO4_CHANNEL   TIM_CHANNEL_4 // PB1 (Aileron kanan)
// Batas Lebar Pulsa PWM Servo (dalam mikrodetik)
#define SERVO_PWM_MIN         1000
#define SERVO_PWM_NEUTRAL     1500
#define SERVO_PWM_MAX         2000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
