/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_audio_if.c
  * @version        : v2.0_Cube
  * @brief          : Generic media access layer.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */
#include "main.h"
#include "arm_math.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

//#define RING_SAMPLES 4096	//リングバッファの要素数

#define UPSAMPLE_FACTOR 2
#define NUM_TAPS 64
#define BLOCK_SIZE 64
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

const float32_t fir_coeffs[64] = {
    -0.00006058f, -0.00012089f, 0.00021086f, 0.00030000f,
    -0.00048500f, -0.00066815f, 0.00098411f, 0.00129652f,
    -0.00180295f, -0.00230366f, 0.00307203f, 0.00383317f,
    -0.00495139f, -0.00606392f, 0.00764078f, 0.00922214f,
    -0.01139796f, -0.01360855f, 0.01657955f, 0.01965930f,
    -0.02373179f, -0.02808442f, 0.03380683f, 0.04021343f,
    -0.04873509f, -0.05898898f, 0.07325762f, 0.09262076f,
    -0.12304609f, -0.17577829f, 0.29776971f, 0.89932368f,
    0.89932368f, 0.29776971f, -0.17577829f, -0.12304609f,
    0.09262076f, 0.07325762f, -0.05898898f, -0.04873509f,
    0.04021343f, 0.03380683f, -0.02808442f, -0.02373179f,
    0.01965930f, 0.01657955f, -0.01360855f, -0.01139796f,
    0.00922214f, 0.00764078f, -0.00606392f, -0.00495139f,
    0.00383317f, 0.00307203f, -0.00230366f, -0.00180295f,
    0.00129652f, 0.00098411f, -0.00066815f, -0.00048500f,
    0.00030000f, 0.00021086f, -0.00012089f, -0.00006058f,
};

// DSPインスタンス（LとRで完全に独立させる）
arm_fir_interpolate_instance_f32 S_Left;
arm_fir_interpolate_instance_f32 S_Right;

// 状態保存用バッファ（CMSIS-DSPの仕様に基づくサイズ計算）
#define STATE_SIZE ((NUM_TAPS / UPSAMPLE_FACTOR) + BLOCK_SIZE - 1)
float32_t fir_state_L[STATE_SIZE];
float32_t fir_state_R[STATE_SIZE];

// 変換用の中間バッファ
float32_t float_in_L[BLOCK_SIZE];
float32_t float_in_R[BLOCK_SIZE];
float32_t float_out_L[BLOCK_SIZE * UPSAMPLE_FACTOR];
float32_t float_out_R[BLOCK_SIZE * UPSAMPLE_FACTOR];

extern int16_t ringbuf[];	//リングバッファ
volatile uint8_t usb_audio_muted = 0U;
extern volatile uint32_t write_pos;	//現在書き込み量
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_AUDIO_IF
  * @{
  */

/** @defgroup USBD_AUDIO_IF_Private_TypesDefinitions USBD_AUDIO_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Defines USBD_AUDIO_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Macros USBD_AUDIO_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Variables USBD_AUDIO_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_FunctionPrototypes USBD_AUDIO_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_FS(uint32_t options);
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol);
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_GetState_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_FS =
{
  AUDIO_Init_FS,
  AUDIO_DeInit_FS,
  AUDIO_AudioCmd_FS,
  AUDIO_VolumeCtl_FS,
  AUDIO_MuteCtl_FS,
  AUDIO_PeriodicTC_FS,
  AUDIO_GetState_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the AUDIO media low layer over USB FS IP
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
  /* USER CODE BEGIN 0 */
  UNUSED(AudioFreq);
  UNUSED(Volume);
  UNUSED(options);
  return (USBD_OK);
  /* USER CODE END 0 */
}

/**
  * @brief  De-Initializes the AUDIO media low layer
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_DeInit_FS(uint32_t options)
{
  /* USER CODE BEGIN 1 */
  UNUSED(options);
  return (USBD_OK);
  /* USER CODE END 1 */
}

/**
  * @brief  Handles AUDIO command.
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 2 */
	switch(cmd)
	  {
	    case AUDIO_CMD_PLAY:
	      /* Reset all stream history.  Stale FIR state at the next start creates
	         a transient even when the ring buffer itself is cleared. */
	      HAL_GPIO_WritePin(XSMT_GPIO_Port, XSMT_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(Amp_SHDN_GPIO_Port, Amp_SHDN_Pin, GPIO_PIN_RESET);
	      memset(ringbuf, 0, sizeof(int16_t) * RING_SAMPLES);
	      memset(fir_state_L, 0, sizeof(fir_state_L));
	      memset(fir_state_R, 0, sizeof(fir_state_R));
	      write_pos = 0U;
	      read_pos = 0U;
	      AudioPipeline_Reset();

	      HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
	      break;

	    case AUDIO_CMD_STOP:
	      // 1. 即座にハードウェアミュート（ノイズ遮断）
	      HAL_GPIO_WritePin(XSMT_GPIO_Port, XSMT_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(Amp_SHDN_GPIO_Port, Amp_SHDN_Pin, GPIO_PIN_RESET);

	      // 2. 再生中LEDを消灯
	      HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

	      // 3. バッファに残ったゴミデータを消去
	      memset(ringbuf, 0, sizeof(int16_t) * RING_SAMPLES);
	      write_pos = 0;
	      read_pos = 0U;
	      AudioPipeline_Reset();
	      break;
	  }

	  UNUSED(pbuf);
	  UNUSED(size);
	  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Controls AUDIO Volume.
  * @param  vol: volume level (0..100)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol)
{
  /* USER CODE BEGIN 3 */
  UNUSED(vol);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  Controls AUDIO Mute.
  * @param  cmd: command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd)
{
  /* USER CODE BEGIN 4 */
  usb_audio_muted = (cmd != 0U) ? 1U : 0U;

  if (usb_audio_muted != 0U)
  {
    HAL_GPIO_WritePin(XSMT_GPIO_Port, XSMT_Pin, GPIO_PIN_RESET);
  }
  else if (audio_started != 0U)
  {
    HAL_GPIO_WritePin(XSMT_GPIO_Port, XSMT_Pin, GPIO_PIN_SET);
  }
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  AUDIO_PeriodicT_FS
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 5 */
  uint32_t wp;
  uint32_t used;
  uint32_t sample_count;

  UNUSED(cmd);

  /* UAC1 stereo/16-bit packets always contain complete 4-byte frames. */
  if ((pbuf == NULL) || ((size & 3U) != 0U))
  {
    return (USBD_FAIL);
  }

  sample_count = size / sizeof(int16_t);
  wp = write_pos;
  used = wp - read_pos;

  /* Never publish or overwrite a partial block.  If the host outruns us,
     dropping one complete packet is safer than corrupting a DMA read in
     progress; explicit feedback will pull the host rate back down. */
  if ((used > RING_SAMPLES) || (sample_count > (RING_SAMPLES - used)))
  {
    audio_overrun_count++;
    return (USBD_BUSY);
  }

  for (uint32_t i = 0U; i < size; i += 4U)
  {
    ringbuf[wp & (RING_SAMPLES - 1U)] =
        (int16_t)((uint16_t)pbuf[i] | ((uint16_t)pbuf[i + 1U] << 8));
    wp++;
    ringbuf[wp & (RING_SAMPLES - 1U)] =
        (int16_t)((uint16_t)pbuf[i + 2U] | ((uint16_t)pbuf[i + 3U] << 8));
    wp++;
  }

  /* Commit once, after the complete packet is visible to the DMA consumer. */
  __DMB();
  write_pos = wp;
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Gets AUDIO State.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_GetState_FS(void)
{
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  Manages the DMA full transfer complete event.
  * @retval None
  */
void TransferComplete_CallBack_FS(void)
{
  /* USER CODE BEGIN 7 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_FULL);
  /* USER CODE END 7 */
}

/**
  * @brief  Manages the DMA Half transfer complete event.
  * @retval None
  */
void HalfTransfer_CallBack_FS(void)
{
  /* USER CODE BEGIN 8 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_HALF);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
void DSP_Process_Upsample(int16_t *pIn_48k, int32_t *pOut_96k_32)
{
	for (int i = 0; i < BLOCK_SIZE; i++) {
		float_in_L[i] = (float32_t)pIn_48k[i * 2];
		float_in_R[i] = (float32_t)pIn_48k[i * 2 + 1];
	}

	arm_fir_interpolate_f32(&S_Left, float_in_L, float_out_L, BLOCK_SIZE);
	arm_fir_interpolate_f32(&S_Right, float_in_R, float_out_R, BLOCK_SIZE);

	for (int i = 0; i < (BLOCK_SIZE * 2); i++) {
		// 1. floatの状態で計算
		float32_t outL = float_out_L[i] * 65536.0f * 0.3f;
		float32_t outR = float_out_R[i] * 65536.0f * 0.3f;

		// 2. 32bit整数の限界でクランプ（飽和演算）
		if (outL > 2147483647.0f)  outL = 2147483647.0f;
		if (outL < -2147483648.0f) outL = -2147483648.0f;
		if (outR > 2147483647.0f)  outR = 2147483647.0f;
		if (outR < -2147483648.0f) outR = -2147483648.0f;

		pOut_96k_32[i * 2]     = (int32_t)outL;
		pOut_96k_32[i * 2 + 1] = (int32_t)outR;
	}
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
