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
#include "stm32l4xx_hal.h"

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
#define Amp_SHDN_Pin GPIO_PIN_0
#define Amp_SHDN_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_10
#define LED_R_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_11
#define LED_B_GPIO_Port GPIOB
#define XSMT_Pin GPIO_PIN_4
#define XSMT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define AUDIO_SAMPLES 512        // SAI書き込み量
#define RING_SAMPLES  8192       // リングバッファサイズ

extern __attribute__((aligned(4))) int32_t audio_buf[AUDIO_SAMPLES]; //DMA用
extern __attribute__((aligned(4))) int16_t ringbuf[RING_SAMPLES]; //リングバッファ
extern volatile uint32_t write_pos;
extern volatile uint32_t read_pos;
extern uint8_t audio_started;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
