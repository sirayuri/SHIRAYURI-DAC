/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <arm_math.h>
#include <usbd_audio_if.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_b;

/* USER CODE BEGIN PV */

int32_t audio_buf[AUDIO_SAMPLES];	//DMAに書き込むよう
int16_t ringbuf[RING_SAMPLES];	//リングバッファ
volatile uint32_t write_pos = 0;
volatile uint32_t read_pos = 0;
volatile uint8_t audio_started = 0;
volatile uint32_t audio_underrun_count = 0;
volatile uint32_t audio_overrun_count = 0;
uint8_t is_playing = 0;
uint32_t silence_cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SAI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define AUDIO_FADE_STEP_Q15  64U
#define AUDIO_RAMP_DOWN_STEP_Q15 (32768U / (AUDIO_SAMPLES / 4U))

static uint16_t audio_gain_q15 = 0U;
static int32_t last_sample_l = 0;
static int32_t last_sample_r = 0;
static uint32_t preroll_samples = AUDIO_PREROLL_SAMPLES;

void AudioPipeline_Reset(void)
{
  audio_gain_q15 = 0U;
  last_sample_l = 0;
  last_sample_r = 0;
  preroll_samples = AUDIO_PREROLL_SAMPLES;
  audio_started = 0U;
  audio_underrun_count = 0U;
  audio_overrun_count = 0U;
}

static int32_t ScaleSampleQ15(int32_t sample, uint32_t gain_q15)
{
  return (int32_t)(((int64_t)sample * (int64_t)gain_q15) >> 15);
}

static void FillRampToSilence(int32_t *dst, uint32_t samples)
{
  uint32_t frames = samples / 2U;
  uint32_t gain_q15 = 32767U;

  for (uint32_t i = 0U; i < frames; i++)
  {
    dst[i * 2U] = ScaleSampleQ15(last_sample_l, gain_q15);
    dst[i * 2U + 1U] = ScaleSampleQ15(last_sample_r, gain_q15);
    gain_q15 = (gain_q15 > AUDIO_RAMP_DOWN_STEP_Q15) ?
               (gain_q15 - AUDIO_RAMP_DOWN_STEP_Q15) : 0U;
  }

  last_sample_l = 0;
  last_sample_r = 0;
  audio_gain_q15 = 0U;
  audio_started = 0U;
  preroll_samples = AUDIO_RECOVERY_SAMPLES;
}

void FillFromRing(int32_t *dst, uint32_t samples)
{
  const uint32_t req_48k_samples = samples / 2U;
  const uint32_t wp = write_pos;
  uint32_t rp = read_pos;
  const uint32_t stored = wp - rp;

  /* A corrupted/overrun state is recovered without reading overwritten data. */
  if (stored > RING_SAMPLES)
  {
    read_pos = wp;
    FillRampToSilence(dst, samples);
    return;
  }

  /* Do not start from an almost-empty buffer.  The pre-roll absorbs Android
     scheduler jitter and the USB/SAI clock-domain phase difference. */
  if ((audio_started == 0U) && (stored < preroll_samples))
  {
    memset(dst, 0, samples * sizeof(int32_t));
    return;
  }

  if (stored >= req_48k_samples)
  {
    int16_t temp_48k[AUDIO_SAMPLES / 2U];
    const uint8_t starting = (audio_started == 0U);

    for (uint32_t i = 0U; i < req_48k_samples; i++)
    {
      temp_48k[i] = ringbuf[rp & (RING_SAMPLES - 1U)];
      rp++;
    }

    /* Publish the consumer position only after the complete stereo block has
       been copied.  This prevents USB from overwriting a half-read frame. */
    __DMB();
    read_pos = rp;

    DSP_Process_Upsample(temp_48k, dst);

    if (starting != 0U)
    {
      audio_started = 1U;
      audio_gain_q15 = 0U;
      preroll_samples = AUDIO_PREROLL_SAMPLES;
      if (usb_audio_muted == 0U)
      {
        HAL_GPIO_WritePin(XSMT_GPIO_Port, XSMT_Pin, GPIO_PIN_SET);
      }
      HAL_GPIO_WritePin(Amp_SHDN_GPIO_Port, Amp_SHDN_Pin, GPIO_PIN_SET);
    }

    /* Short de-zipper fade after start/recovery. */
    if (audio_gain_q15 < 32767U)
    {
      for (uint32_t i = 0U; i < samples; i += 2U)
      {
        uint32_t next_gain = audio_gain_q15 + AUDIO_FADE_STEP_Q15;
        audio_gain_q15 = (uint16_t)((next_gain > 32767U) ? 32767U : next_gain);
        dst[i] = ScaleSampleQ15(dst[i], audio_gain_q15);
        dst[i + 1U] = ScaleSampleQ15(dst[i + 1U], audio_gain_q15);
      }
    }

    last_sample_l = dst[samples - 2U];
    last_sample_r = dst[samples - 1U];
  }
  else
  {
    /* A hard transition to zero is audible as a click.  Conceal an underrun
       with a one-DMA-half ramp, then wait for a fresh pre-roll. */
    audio_underrun_count++;
    FillRampToSilence(dst, samples);
  }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  UNUSED(hsai);
  FillFromRing(&audio_buf[0], AUDIO_SAMPLES / 2U);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  UNUSED(hsai);
  FillFromRing(&audio_buf[AUDIO_SAMPLES / 2U], AUDIO_SAMPLES / 2U);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SAI1_Init();
  /* USER CODE BEGIN 2 */
  extern arm_fir_interpolate_instance_f32 S_Left;
  extern arm_fir_interpolate_instance_f32 S_Right;
  extern const float32_t fir_coeffs[];
  extern float32_t fir_state_L[];
  extern float32_t fir_state_R[];

  arm_fir_interpolate_init_f32(&S_Left, 2, 64, (float32_t*)fir_coeffs, fir_state_L, 64);
  arm_fir_interpolate_init_f32(&S_Right, 2, 64, (float32_t*)fir_coeffs, fir_state_R, 64);

  //バッファをクリア
  memset(audio_buf, 0, sizeof(audio_buf));
  memset(ringbuf, 0, sizeof(ringbuf));

  //DMAスタート
  if (HAL_SAI_Transmit_DMA(&hsai_BlockB1, (uint8_t*)audio_buf,
                           AUDIO_SAMPLES) != HAL_OK)
  {
    Error_Handler();
  }

  /* Keep the analog path muted until USB has provided a complete pre-roll.
     FillFromRing() enables it immediately before the click-free fade-in. */
  HAL_GPIO_WritePin(XSMT_GPIO_Port, XSMT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Amp_SHDN_GPIO_Port, Amp_SHDN_Pin, GPIO_PIN_RESET);

  /* Expose USB only after SAI, DMA, FIR state and buffers are ready. */
  MX_USB_DEVICE_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    __WFI();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
  hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_96K;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.FrameInit.FrameLength = 64;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockB1.SlotInit.SlotNumber = 2;
  hsai_BlockB1.SlotInit.SlotActive = 0x00000003;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel2_IRQn interrupt configuration */
  /* Keep USB and SAI DMA at the same highest preemption level.  This prevents
     PLAY/STOP from preempting a FIR update while still servicing both without
     lower-priority application interrupt latency. */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Amp_SHDN_GPIO_Port, Amp_SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_R_Pin|LED_B_Pin|XSMT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Amp_SHDN_Pin */
  GPIO_InitStruct.Pin = Amp_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Amp_SHDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_B_Pin XSMT_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_B_Pin|XSMT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
