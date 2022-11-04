/**
******************************************************************************
* @file    BlueCoin_audio.c
* @author  SRA - Central Labs
* @version v2.2.3
* @date    10-Feb-2022
* @brief   This file provides the Audio driver for the BLUECOIN
*          board.
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

/* Includes ------------------------------------------------------------------*/
#include "BlueCoin_audio.h"
#include "BlueCoin_conf.h"
#include "audio.h"

#include "arm_math.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup BLUECOIN
  * @{
  */ 
  
/** @defgroup BLUECOIN_AUDIO BLUECOIN AUDIO
  * @{
  */ 
/** @defgroup BLUECOIN_AUDIO_Private_Defines BLUECOIN_AUDIO Private Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup BLUECOIN_AUDIO_Private_Macros BLUECOIN_AUDIO Private Macros
  * @{
  */

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define SAI_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (12U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (2U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (6U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (1U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (3U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (0U) \
         : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (2U) : (1U)

#define DFSDM_OVER_SAMPLING(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (64U)  \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (32U) : (32U) 
	  
#define DFSDM_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (17U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (17U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (16U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (16U) : (16U)

#define DFSDM_FILTER_ORDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (DFSDM_FILTER_SINC5_ORDER) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (DFSDM_FILTER_SINC5_ORDER) : (DFSDM_FILTER_SINC5_ORDER)

#define DFSDM_MIC_BIT_SHIFT(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (5U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (10U) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (5U) : (5U)

/**
  * @}
  */ 

/** @defgroup BLUECOIN_AUDIO_Exported_Variables BLUECOIN_AUDIO Exported Variables
  * @{
  */
/* Recording context */
AUDIO_IN_Ctx_t                         AudioInCtx[AUDIO_IN_INSTANCES_NBR] = {0};
AUDIO_OUT_Ctx_t                        AudioOutCtx[AUDIO_OUT_INSTANCES_NBR] = {0};

/**
  * @}
  */
  
/** @defgroup BLUECOIN_AUDIO_Private_Variables BLUECOIN_AUDIO Private Variables
  * @{
  */
static AUDIO_Drv_t                     *AudioDrv = NULL;
static void                            *CompObj = NULL;

/* Play handle */
SAI_HandleTypeDef               hAudioOutSai;

/* Recording handles */
#define DECIMATOR_NUM_TAPS 16
#define DECIMATOR_BLOCK_SIZE 16 
#define DECIMATOR_FACTOR 2
#define DECIMATOR_STATE_LENGTH (DECIMATOR_BLOCK_SIZE + (DECIMATOR_NUM_TAPS) -1)
static arm_fir_decimate_instance_q15 ARM_Decimator_State[4];

I2S_HandleTypeDef               hAudioInI2s;
static SPI_HandleTypeDef               hAudioInSPI;

static TIM_HandleTypeDef        TimDividerHandle;

static uint16_t I2S_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_I2S];
static uint16_t SPI_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_SPI];

/* PDM filters params */
static PDM_Filter_Handler_t  PDM_FilterHandler[4];
static PDM_Filter_Config_t   PDM_FilterConfig[4];
            
static uint8_t Channel_Demux[128] = {
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f
};


/**
  * @}
  */ 

/** @defgroup BLUECOIN_AUDIO_Private_Function_Prototypes BLUECOIN_AUDIO Private Function Prototypes
  * @{
  */
static HAL_StatusTypeDef AUDIO_IN_Timer_Init(void);
static HAL_StatusTypeDef AUDIO_IN_Timer_Start(void);
static void I2S_MspInit(I2S_HandleTypeDef *hi2s);
static void SPI_MspInit(SPI_HandleTypeDef *hspi);

/* SAI Msp config */
static void SAI_MspInit(SAI_HandleTypeDef *hsai);
static void SAI_MspDeInit(SAI_HandleTypeDef *hsai);
static void SAI_InitMXConfigStruct(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig);

/*   */
static int32_t PCM1774_Probe(void);
/**
  * @}
  */ 


/** @defgroup STM32H747I_EVAL_AUDIO_OUT_Private_Functions STM32H747I_EVAL_AUDIO_OUT Private Functions
  * @{
  */ 
/**
  * @brief  Configures the audio peripherals.
* @param  Instance   AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  AudioInit  AUDIO OUT init Structure
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit)
{
 if(Instance >= AUDIO_OUT_INSTANCES_NBR)
 {
   return BSP_ERROR_WRONG_PARAM; 
 }
 else
 {  
   /* Fill AudioOutCtx structure */
   AudioOutCtx[Instance].Device         = AudioInit->Device;
   AudioOutCtx[Instance].Instance       = Instance; 
   AudioOutCtx[Instance].SampleRate     = AudioInit->SampleRate;
   AudioOutCtx[Instance].BitsPerSample  = AudioInit->BitsPerSample;
   AudioOutCtx[Instance].ChannelsNbr    = AudioInit->ChannelsNbr;
   AudioOutCtx[Instance].Volume         = AudioInit->Volume;
   AudioOutCtx[Instance].State          = AUDIO_OUT_STATE_RESET;
   
   if (AudioOutCtx[Instance].SampleRate != AUDIO_FREQUENCY_8K && AudioOutCtx[Instance].SampleRate != AUDIO_FREQUENCY_16K 
       && AudioOutCtx[Instance].SampleRate && AudioOutCtx[Instance].SampleRate != AUDIO_FREQUENCY_32K 
         && AudioOutCtx[Instance].SampleRate != AUDIO_FREQUENCY_48K)
   {
    return BSP_ERROR_WRONG_PARAM;
   }
   
   /* PLL clock SETUP */ 
   if(MX_SAI1_ClockConfig(&hAudioOutSai, AudioOutCtx[Instance].SampleRate) != HAL_OK)
   {
     return BSP_ERROR_CLOCK_FAILURE;
   }
   /* SAI data transfer preparation:
   Prepare the Media to be used for the audio transfer from memory to SAI peripheral */
   hAudioOutSai.Instance = AUDIO_SAIx;

   SAI_MspInit(&hAudioOutSai);
   
   MX_SAI_Config mx_sai_config;
   
   /* Prepare hAudioOutSai handle */
   mx_sai_config.AudioFrequency    = AudioInit->SampleRate;
   mx_sai_config.AudioMode         = SAI_MODEMASTER_TX;
   mx_sai_config.ClockSource       = SAI_CLKSOURCE_PLLSAI;
   mx_sai_config.ClockStrobing     = SAI_CLOCKSTROBING_RISINGEDGE;
   mx_sai_config.MonoStereoMode    = SAI_STEREOMODE;
   mx_sai_config.DataSize          = SAI_DATASIZE_16;
   mx_sai_config.FrameLength       = 32; 
   mx_sai_config.ActiveFrameLength = 16;  
   mx_sai_config.OutputDrive       = SAI_OUTPUTDRIVE_ENABLED;
   mx_sai_config.Synchro           = SAI_ASYNCHRONOUS;
   mx_sai_config.SlotActive         = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;
   
   /* SAI peripheral initialization: this __weak function can be redefined by the application  */
   if(MX_SAI1_Block_A_Init(&hAudioOutSai, &mx_sai_config) != HAL_OK)
   {
     return BSP_ERROR_PERIPH_FAILURE;
   }
      
   if(PCM1774_Probe()!= BSP_ERROR_NONE)
   {
     return BSP_ERROR_COMPONENT_FAILURE;
   } 
   
   /* Update BSP AUDIO OUT state */
   AudioOutCtx[Instance].State = AUDIO_OUT_STATE_STOP;
  }
  
  return BSP_ERROR_NONE; 
}

/**
  * @brief  De-initializes the audio out peripheral.
* @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval None
  */
int32_t BSP_AUDIO_OUT_DeInit(uint32_t Instance)
{  
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM; 
  }
  else
  {
    SAI_MspDeInit(&hAudioOutSai);
    /* Initialize the hAudioOutSai Instance parameter */
    hAudioOutSai.Instance = AUDIO_SAIx;
    /* Call the Media layer stop function */
    if(AudioDrv->DeInit(CompObj) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }   
    else if(HAL_SAI_DeInit(&hAudioOutSai) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }   
    else
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_RESET;    
    } 
  }
  /* Return BSP status */
  return ret;  
}
 
/**
  * @brief  Initializes the Audio Codec audio out instance (SAI).
  * @param  MXConfig SAI configuration structure
  * @note   Being __weak it can be overwritten by the application
  * @retval HAL status
  */
__weak HAL_StatusTypeDef MX_SAI1_Block_A_Init(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig)
{ 
  HAL_StatusTypeDef ret = HAL_OK;
  
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(hsai);
  
  /* Configure SAI1_Block_A */
  hsai->Init.MonoStereoMode       = MXConfig->MonoStereoMode;
  hsai->Init.AudioFrequency       = MXConfig->AudioFrequency;
  hsai->Init.AudioMode            = MXConfig->AudioMode;
  hsai->Init.NoDivider            = SAI_MASTERDIVIDER_ENABLE;
  hsai->Init.Protocol             = SAI_FREE_PROTOCOL;
  hsai->Init.DataSize             = MXConfig->DataSize;
  hsai->Init.FirstBit             = SAI_FIRSTBIT_MSB;
  hsai->Init.ClockStrobing        = MXConfig->ClockStrobing;
  hsai->Init.Synchro              = MXConfig->Synchro;
  hsai->Init.OutputDrive          = MXConfig->OutputDrive;
  hsai->Init.FIFOThreshold        = SAI_FIFOTHRESHOLD_1QF;
  hsai->Init.ClockSource          = MXConfig->ClockSource;
  
  /* Configure SAI_Block_x Frame */
  hsai->FrameInit.FrameLength       = MXConfig->FrameLength; 
  hsai->FrameInit.ActiveFrameLength = MXConfig->ActiveFrameLength;  
  hsai->FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai->FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  hsai->FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;
  
  /* Configure SAI Block_x Slot */
  hsai->SlotInit.FirstBitOffset     = 0;
  hsai->SlotInit.SlotSize           = SAI_SLOTSIZE_16B;
  hsai->SlotInit.SlotNumber         = 2;  
  hsai->SlotInit.SlotActive         = MXConfig->SlotActive;
  
  if(HAL_SAI_Init(hsai) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  
  __HAL_SAI_ENABLE(hsai);
  
  return ret;
}

/**
  * @brief  SAI clock Config.
  * @param  hsai SAI handle
  * @param  SampleRate  Audio frequency used to play the audio stream.
  * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
  *         Being __weak it can be overwritten by the application     
  * @retval HAL status
  */
__weak HAL_StatusTypeDef MX_SAI1_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t SampleRate)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);
  UNUSED(SampleRate);
  
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  
  /* Configure PLLSAI prescalers */
  /* PLLSAI_VCO: VCO_429M
     SAI_CLK(first level) = PLLSAI_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
     SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ = 214.5/19 = 11.289 Mhz */
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  RCC_PeriphCLKInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIM = 16;
  RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIN = 344;
  RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIQ = 7;
  RCC_PeriphCLKInitStruct.PLLSAIDivQ = 1;
  
  if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  
  return ret;
}


/**
  * @brief  Starts playing audio stream from a data buffer for a determined size.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  pData         pointer on data address 
  * @param  NbrOfBytes   Size of total samples in bytes
  *                      BitsPerSample: 16 or 32
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Play(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance >= AUDIO_OUT_INSTANCES_NBR) || (((NbrOfBytes / (AudioOutCtx[Instance].BitsPerSample/8U)) > 0xFFFFU)))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if((AudioOutCtx[Instance].State == AUDIO_OUT_STATE_STOP) || (AudioOutCtx[Instance].State == AUDIO_OUT_STATE_RESET))
  {
    if(HAL_SAI_Transmit_DMA(&hAudioOutSai, pData, DMA_MAX(NbrOfBytes)) != HAL_OK)
    {  
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_PLAYING;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret;  
}

/**
  * @brief  This function Pauses the audio file stream. In case
  *         of using DMA, the DMA Pause feature is used.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @note   When calling BSP_AUDIO_OUT_Pause() function for pause, only
  *          BSP_AUDIO_OUT_Resume() function should be called for resume (use of BSP_AUDIO_OUT_Play() 
  *          function for resume could lead to unexpected behavior).
  * @retval BSP status
  */  
int32_t BSP_AUDIO_OUT_Pause(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Call the Media layer pause function */   
    if(HAL_SAI_DMAPause(&hAudioOutSai) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }    
    
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_PAUSE;   
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief   Resumes the audio file stream.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @note    When calling BSP_AUDIO_OUT_Pause() function for pause, only
  *          BSP_AUDIO_OUT_Resume() function should be called for resume (use of BSP_AUDIO_OUT_Play() 
  *          function for resume could lead to unexpected behavior).
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Resume(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Call the Media layer pause/resume function */
    if(HAL_SAI_DMAResume(&hAudioOutSai) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    
    if(ret == BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_PLAYING;
    }
  }
  
  /* Return BSP status */
  return ret;  
}

/**
  * @brief  Stops audio playing and Power down the Audio Codec.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Stop(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (AudioOutCtx[Instance].State == AUDIO_OUT_STATE_PLAYING)
  {
    /* Call the Media layer stop function */
    if(AudioDrv->Stop(CompObj, CODEC_PDWN_SW) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      if(HAL_SAI_DMAStop(&hAudioOutSai)!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }

      if( ret==BSP_ERROR_NONE)
      {
        /* Update BSP AUDIO OUT state */    
        AudioOutCtx[Instance].State = AUDIO_OUT_STATE_STOP;  
      }
    }    
  }  
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret;
}

/**
  * @brief  Controls the current audio volume level.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  Volume    Volume level to be set in percentage from 0% to 100% (0 for 
  *         Mute and 100 for Max volume level).
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t ret = BSP_ERROR_NONE;

  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {      
    /* Call the codec volume control function with converted volume value */
    if(AudioDrv->SetVolume(CompObj, 1, VOLUME_OUT_CONVERT(Volume)) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if(Volume == 0U)
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = BSP_AUDIO_MUTE_ENABLED;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = BSP_AUDIO_MUTE_DISABLED;      
    }
   AudioOutCtx[Instance].Volume = Volume;
  }

  /* Return BSP status */
  return ret;  
}

/**
  * @brief  Get the current audio volume level.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  Volume    pointer to volume to be returned
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *Volume = AudioOutCtx[Instance].Volume;
  }
  /* Return BSP status */
  return ret;  
}

/**
  * @brief  Enables the MUTE
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_Mute(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  { 
    ret = BSP_ERROR_WRONG_PARAM;    
  }
  else
  {
    /* Call the Codec Mute function */
    if(AudioDrv->SetMute(CompObj, CODEC_MUTE_ON) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = BSP_AUDIO_MUTE_ENABLED;
    }
  }
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Disables the MUTE mode
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_UnMute(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;    
  }
  else
  {    
    /* Call the Codec Mute function */
    if(AudioDrv->SetMute(CompObj, CODEC_MUTE_OFF) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = BSP_AUDIO_MUTE_DISABLED;
    }
  }  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Check whether the MUTE mode is enabled or not
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  IsMute    pointer to mute state
  * @retval Mute status
  */
int32_t BSP_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *IsMute = AudioOutCtx[Instance].IsMute; 
  }
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Switch dynamically (while audio file is played) the output target 
  *         (speaker or headphone).
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  Device  The audio output device
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t ret;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  { 
    /* Call the Codec output device function */
    if(AudioDrv->SetOutputMode(CompObj, Device) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      MX_SAI_Config mx_out_config;
      /* Get SAI MX configuration */
      SAI_InitMXConfigStruct(&hAudioOutSai, &mx_out_config);
      if(MX_SAI1_Block_A_Init(&hAudioOutSai, &mx_out_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        /* Update AudioOutCtx structure */
        AudioOutCtx[Instance].Device = Device;
      }
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Get the Output Device 
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  Device    The audio output device
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  { 
    /* Get AudioOutCtx Device */
    *Device = AudioOutCtx[Instance].Device;
  } 
  /* Return BSP status */
  return ret;   
}

/**
  * @brief  Updates the audio frequency.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  SampleRate Audio frequency used to play the audio stream.
  * @note   This API should be called after the BSP_AUDIO_OUT_Init() to adjust the
  *         audio frequency.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    /* Call the Codec output device function */
    if(AudioDrv->SetFrequency(CompObj, SampleRate) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }    
    else
    {
      MX_SAI_Config mx_sai_config;
      
      /* Get SAI MX configuration */
      SAI_InitMXConfigStruct(&hAudioOutSai, &mx_sai_config);
      /* Update MX config structure */
      mx_sai_config.Mckdiv = SAI_CLOCK_DIVIDER(SampleRate);
      /* Update the SAI audio frequency configuration */
      hAudioOutSai.Init.Mckdiv = SAI_CLOCK_DIVIDER(SampleRate);
      /* PLL clock setup */ 
      if(MX_SAI1_ClockConfig(&hAudioOutSai, SampleRate) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else if(MX_SAI1_Block_A_Init(&hAudioOutSai, &mx_sai_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {        
        /* Store new sample rate */
        AudioOutCtx[Instance].SampleRate = SampleRate;     
      }
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY; 
  }
  
  /* Return BSP status */
  return ret;   
}

/**
  * @brief  Get the audio frequency.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  SampleRate  Audio frequency used to play the audio stream.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    *SampleRate = AudioOutCtx[Instance].SampleRate;
  }   
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Get the audio Resolution.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  BitsPerSample  Audio Resolution used to play the audio stream.
  * @retval BSP status
  */

int32_t BSP_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    if (BitsPerSample != AUDIO_RESOLUTION_16b)      
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }    
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret; 
}    

/**
  * @brief  Get the audio Resolution.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  BitsPerSample  Audio Resolution used to play the audio stream.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Get audio Out resolution */
    *BitsPerSample = AudioOutCtx[Instance].BitsPerSample;    
  }
  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Set the audio Channels number.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  ChannelNbr  Audio Channels number used to play the audio stream.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance >= AUDIO_OUT_INSTANCES_NBR) || (ChannelNbr > 2U))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    /* Set the audio Channels number */
    if(ChannelNbr != 2U)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {    
    /* Store new Channel number */
    AudioOutCtx[Instance].ChannelsNbr = ChannelNbr;         
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Get the audio Channels number.
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  ChannelNbr     Audio Channels number used to play the audio stream.
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Get the audio Channels number */
    *ChannelNbr = AudioOutCtx[Instance].ChannelsNbr;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
  * @brief  Get Audio Out state
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @param  State     Audio Out state
  * @retval BSP status
  */
int32_t BSP_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
  /* Return audio Output State */
  *State = AudioOutCtx[Instance].State;
  }
  
  /* Return BSP status */  
  return ret;
}


/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hsai SAI handle
  * @retval None
  */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);
  
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32h747i_eval_audio.h) */
  BSP_AUDIO_OUT_TransferComplete_CallBack(0);
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hsai  SAI handle
  * @retval None
  */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);
  
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32h747i_eval_audio.h) */
  BSP_AUDIO_OUT_HalfTransfer_CallBack(0);
}

/**
  * @brief  SAI error callbacks.
  * @param  hsai  SAI handle
  * @retval None
  */
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);
  BSP_AUDIO_OUT_Error_CallBack(0);
 
}


/**
  * @brief  Manages the DMA full Transfer complete event
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval None
  */
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);  
}

/**
  * @brief  Manages the DMA Half Transfer complete event
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval None
  */
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);  
}

/**
  * @brief  Manages the DMA FIFO error event
  * @param AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval None
  */
__weak void BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);  
}

/** @defgroup BLUECOIN_AUDIO_IN_Private_Functions BLUECOIN_AUDIO_IN Private Functions
  * @{
  */ 
  
/**
* @brief  Initialize wave recording.
* @param  Instance  AUDIO IN Instance. It can be:
*       - 0 when I2S is used 
*       - 1 if DFSDM is used
*       - 2 if PDM is used
* @param  AudioInit Init structure
* @retval BSP status
*/
__weak int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit)
{
  int32_t ret =  BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    /* Store the audio record context */
    AudioInCtx[Instance].Device          = AudioInit->Device;
    AudioInCtx[Instance].ChannelsNbr     = AudioInit->ChannelsNbr;  
    AudioInCtx[Instance].SampleRate      = AudioInit->SampleRate; 
    AudioInCtx[Instance].BitsPerSample   = AudioInit->BitsPerSample;
    AudioInCtx[Instance].Volume          = AudioInit->Volume;
    AudioInCtx[Instance].State           = AUDIO_IN_STATE_RESET;
    
    if(Instance == 0U)
    { 
      uint32_t PDM_Clock_Freq;      
      MX_I2S_Config i2s_config;
      
      switch (AudioInit->SampleRate)
      {
      case AUDIO_FREQUENCY_8K:
        PDM_Clock_Freq = 1024;
        break;
        
      case AUDIO_FREQUENCY_16K:
        PDM_Clock_Freq = 2048;
        break;
        
      case AUDIO_FREQUENCY_32K:
        PDM_Clock_Freq = 2048;
        break;
        
      case AUDIO_FREQUENCY_48K:
        PDM_Clock_Freq = 3072;
        break;
        
      default:
        PDM_Clock_Freq = 1280;
        ret =  BSP_ERROR_WRONG_PARAM;
        break;
      }
      
      AudioInCtx[Instance].DecimationFactor = (PDM_Clock_Freq * 1000U)/AudioInit->SampleRate;
      AudioInCtx[Instance].Size = (PDM_Clock_Freq/8U) * 2U * N_MS_PER_INTERRUPT;
      BSP_Mic234_Clock_Selector_Init();
            
      if(AudioInCtx[0].ChannelsNbr == 1)
      {
        i2s_config.DataFormat   = I2S_DATAFORMAT_16B;
	BSP_Mic234_Clock_Selector_Set(CLK_DISABLE);
      }
      else
      {
        i2s_config.DataFormat   = I2S_DATAFORMAT_32B;
	BSP_Mic234_Clock_Selector_Set(CLK_ENABLE);
      }
      
      i2s_config.AudioFreq = ((PDM_Clock_Freq * 1000U) / 32U);
      i2s_config.CPOL         = I2S_CPOL_HIGH;
      i2s_config.MCLKOutput   = I2S_MCLKOUTPUT_DISABLE;
      i2s_config.Mode         = I2S_MODE_MASTER_RX;
      i2s_config.Standard     = I2S_STANDARD_MSB;
      i2s_config.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
      i2s_config.ClockSource  = I2S_CLOCK_PLL;
      
      if (AudioInCtx[0].ChannelsNbr>1U)
      {
        PDM_Clock_Freq *=2U;
        if (AUDIO_IN_Timer_Init() != HAL_OK)
        {
          ret =  BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      /* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */ 
      if(MX_I2S_ClockConfig(&hAudioInI2s, PDM_Clock_Freq) != HAL_OK)
      {
        ret =  BSP_ERROR_CLOCK_FAILURE;
      }
      
      /* I2S Peripheral configuration */
      hAudioInI2s.Instance          = AUDIO_IN_I2S_INSTANCE;
      __HAL_I2S_DISABLE(&hAudioInI2s);
      I2S_MspInit(&hAudioInI2s);
      
      if (MX_I2S_Init(&hAudioInI2s, &i2s_config)!= HAL_OK)
      {
        ret =  BSP_ERROR_PERIPH_FAILURE;
      }
      if (HAL_I2S_Init(&hAudioInI2s) != HAL_OK)
      {
        ret =  BSP_ERROR_PERIPH_FAILURE;
      }
      
      if (AudioInCtx[0].ChannelsNbr>2U)
      {
        /* Set the SPI parameters */
        hAudioInSPI.Instance               = AUDIO_IN_SPI_INSTANCE;
        
        __HAL_SPI_DISABLE(&hAudioInSPI);
        SPI_MspInit(&hAudioInSPI);
        
        MX_SPI_Config spi_config;
        spi_config.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        spi_config.Direction         = SPI_DIRECTION_2LINES_RXONLY;
        spi_config.CLKPhase          = SPI_PHASE_2EDGE;
        spi_config.CLKPolarity       = SPI_POLARITY_HIGH;
        spi_config.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
        spi_config.CRCPolynomial     = 7;
        spi_config.DataSize          = SPI_DATASIZE_16BIT;
        spi_config.FirstBit          = SPI_FIRSTBIT_MSB;
        spi_config.NSS               = SPI_NSS_SOFT;
        spi_config.TIMode            = SPI_TIMODE_DISABLED;
        spi_config.Mode              = SPI_MODE_SLAVE;
        
        if (MX_SPI_Init(&hAudioInSPI, &spi_config)!= HAL_OK)
        {
          ret =  BSP_ERROR_PERIPH_FAILURE;
        }
        if (HAL_SPI_Init(&hAudioInSPI) != HAL_OK)
        {
          ret =  BSP_ERROR_PERIPH_FAILURE;
        }
        
      }
      
      if (BSP_AUDIO_IN_PDMToPCM_Init(Instance, AudioInCtx[0].SampleRate, AudioInCtx[0].ChannelsNbr, AudioInCtx[0].ChannelsNbr)!= BSP_ERROR_NONE)
      {
        ret =  BSP_ERROR_NO_INIT;
      }
    }
    else if(Instance == 1U)
    {
    ret =  BSP_ERROR_WRONG_PARAM;
    }
    else /* Instance = 2 */
    {      
    
    }
    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP; 
    /* Return BSP status */ 
  }
  return ret;
}

/**
* @brief  Deinit the audio IN peripherals.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/

__weak int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(Instance == 0U)
    {

    }
    
    else /* (Instance == 1U) */
    {
      return BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RESET;   
  }
  /* Return BSP status */
  return ret;
}


/**
* @brief  Clock Config.
* @param  hi2s: I2S handle if required
* @param  SampleRate: Audio frequency used to play the audio stream.
* @note   This API is called by BSP_AUDIO_IN_Init() 
*         Being __weak it can be overwritten by the application     
* @retval HAL_OK if no problem during execution, HAL_ERROR otherwise
*/
__weak HAL_StatusTypeDef MX_I2S_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t PDM_rate)
{ 
  
  HAL_StatusTypeDef ret = HAL_OK;
  /*I2S PLL Configuration*/
  RCC_PeriphCLKInitTypeDef rccclkinit;
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit); 

  rccclkinit.PLLI2S.PLLI2SQ = 2;
  rccclkinit.PLLI2SDivQ = 1;

  if (PDM_rate % 1280 == 0)
  {
    rccclkinit.PLLI2S.PLLI2SM = 10;
    rccclkinit.PLLI2S.PLLI2SN = 192;
    rccclkinit.PLLI2S.PLLI2SR = 5;
  }
  else
  {
    rccclkinit.PLLI2S.PLLI2SM = 16;
    rccclkinit.PLLI2S.PLLI2SN = 344;
    rccclkinit.PLLI2S.PLLI2SR = 7;
  }   
  
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2;
  
  if(HAL_RCCEx_PeriphCLKConfig(&rccclkinit) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  ret = HAL_OK;
  
  return ret;
}


__weak HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef* hspi, MX_SPI_Config *MXConfig)
{  
  static DMA_HandleTypeDef hdma_rx;
  HAL_StatusTypeDef ret = HAL_OK;
       
  hspi->Init.BaudRatePrescaler = MXConfig->BaudRatePrescaler; 
  hspi->Init.Direction         = MXConfig->Direction;         
  hspi->Init.CLKPhase          = MXConfig->CLKPhase;          
  hspi->Init.CLKPolarity       = MXConfig->CLKPolarity;       
  hspi->Init.CRCCalculation    = MXConfig->CRCCalculation;    
  hspi->Init.CRCPolynomial     = MXConfig->CRCPolynomial;     
  hspi->Init.DataSize          = MXConfig->DataSize;          
  hspi->Init.FirstBit          = MXConfig->FirstBit;         
  hspi->Init.NSS               = MXConfig->NSS;               
  hspi->Init.TIMode            = MXConfig->TIMode;            
  hspi->Init.Mode              = MXConfig->Mode; 
  
    /* Configure the DMA handler for Transmission process */
  hdma_rx.Instance                 = AUDIO_IN_SPI_RX_DMA_STREAM;
  hdma_rx.Init.Channel             = AUDIO_IN_SPI_RX_DMA_CHANNEL;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_rx.Init.Mode                = DMA_CIRCULAR;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

  /* Configure the DMA Stream */ 
  HAL_DMA_Init(&hdma_rx);
    
  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(hspi, hdmarx, hdma_rx);      

  return ret;
}


__weak HAL_StatusTypeDef MX_I2S_Init(I2S_HandleTypeDef* hi2s, MX_I2S_Config *MXConfig)
{
  static DMA_HandleTypeDef hdma_i2sRx;
  HAL_StatusTypeDef ret = HAL_OK;
  
  hi2s->Init.DataFormat = MXConfig->DataFormat;
  hi2s->Init.AudioFreq = MXConfig->AudioFreq;
  hi2s->Init.ClockSource = MXConfig->ClockSource;
  hi2s->Init.CPOL = MXConfig->CPOL;
  hi2s->Init.MCLKOutput = MXConfig->MCLKOutput;
  hi2s->Init.Mode = MXConfig->Mode;
  hi2s->Init.Standard = MXConfig->Standard;
  hi2s->Init.FullDuplexMode = MXConfig->FullDuplexMode; 
  
  /* Enable the DMA clock */
  AUDIO_IN_I2S_DMAx_CLK_ENABLE();
  
  if(hi2s->Instance == AUDIO_IN_I2S_INSTANCE)
  {
    /* Configure the hdma_i2sRx handle parameters */
    hdma_i2sRx.Init.Channel             = AUDIO_IN_I2S_DMAx_CHANNEL;
    hdma_i2sRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2sRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sRx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sRx.Init.PeriphDataAlignment = AUDIO_IN_I2S_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sRx.Init.MemDataAlignment    = AUDIO_IN_I2S_DMAx_MEM_DATA_SIZE;
    hdma_i2sRx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2sRx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2sRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_i2sRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2sRx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2sRx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
    
    hdma_i2sRx.Instance = AUDIO_IN_I2S_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmarx, hdma_i2sRx);
    
    /* Deinitialize the Stream for new transfer */
    if (HAL_DMA_DeInit(&hdma_i2sRx) != HAL_OK)
    {
      ret = HAL_ERROR;
    }
    
    /* Configure the DMA Stream */
    if (HAL_DMA_Init(&hdma_i2sRx) != HAL_OK)
    {
      ret = HAL_ERROR;
    }
  }
  else
  {
    ret = HAL_ERROR;
  }
  
  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_IN_I2S_DMAx_IRQ, BSP_AUDIO_IN_IT_PRIORITY, BSP_AUDIO_IN_IT_PRIORITY);
  HAL_NVIC_EnableIRQ(AUDIO_IN_I2S_DMAx_IRQ); 
  
  return ret;
}


/**
* @brief  Initialize the PDM library.
* @param Instance    AUDIO IN Instance
* @param  AudioFreq  Audio sampling frequency
* @param  ChnlNbrIn  Number of input audio channels in the PDM buffer
* @param  ChnlNbrOut Number of desired output audio channels in the  resulting PCM buffer
* @retval BSP status
*/
__weak int32_t BSP_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{
  int32_t ret =  BSP_ERROR_NONE;
  
  if(Instance != 0U)
  {
    ret =  BSP_ERROR_WRONG_PARAM;
  }
  else 
  {   
    uint32_t index;    
    static int16_t aState_ARM[4][DECIMATOR_STATE_LENGTH];
    static int16_t aCoeffs[] = { -1406, 1634, -1943, 2386, -3080, 4325, -7223, 21690, 21690, -7223, 4325, -3080, 2386, -1943, 1634, -1406, };
    
    /* Enable CRC peripheral to unlock the PDM library */
    __HAL_RCC_CRC_CLK_ENABLE();
    
    for(index = 0; index < ChnlNbrIn; index++)
    {
      volatile uint32_t error = 0;
      /* Init PDM filters */
      PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
      PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
      PDM_FilterHandler[index].high_pass_tap = 2122358088;
      PDM_FilterHandler[index].out_ptr_channels = (uint16_t)ChnlNbrOut;
      PDM_FilterHandler[index].in_ptr_channels  = (uint16_t)ChnlNbrIn;
      
      /* PDM lib config phase */
      PDM_FilterConfig[index].output_samples_number = (uint16_t) ((AudioFreq/1000U) * N_MS_PER_INTERRUPT);
      PDM_FilterConfig[index].mic_gain = 24;
      
      switch (AudioInCtx[0].DecimationFactor)
      {
      case 16:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_16;
        break;
      case 24:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_24;
        break;
      case 32:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_32;
        break;
      case 48:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_48;
        break;
      case 64:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
        break;
      case 80:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_80;
        break;
      case 128:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_128;
        break;
      case 160:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_80;
        PDM_FilterConfig[index].output_samples_number *= 2U;        
        PDM_FilterHandler[index].out_ptr_channels = 1;
        (void)arm_fir_decimate_init_q15  (&ARM_Decimator_State[index], DECIMATOR_NUM_TAPS, DECIMATOR_FACTOR,
                                          aCoeffs, aState_ARM[index], DECIMATOR_BLOCK_SIZE);
         break;
      default:
        ret =  BSP_ERROR_WRONG_PARAM;
        break;
      }
      
      error = PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));
      if (error!=0U)
      {
        ret =  BSP_ERROR_NO_INIT;
      }
      
      error = PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
      if (error!=0U)
      {
        ret =  BSP_ERROR_NO_INIT;
      }
    }
  } 
  return ret;
}

        

/**
* @brief  Converts audio format from PDM to PCM.
* @param  Instance  AUDIO IN Instance  
* @param  PDMBuf    Pointer to PDM buffer data
* @param  PCMBuf    Pointer to PCM buffer data
* @retval BSP status
*/
__weak int32_t BSP_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf)
{  
  int32_t ret =  BSP_ERROR_NONE;
  
  if(Instance != 0U)
  {
    ret =  BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    uint32_t index;
    
    for(index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
      if (AudioInCtx[0].DecimationFactor == 160)
      {
        uint16_t Decimate_Out[8];
        uint32_t ii;
        uint16_t PDM_Filter_Out[16];
        
        (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], PDM_Filter_Out, &PDM_FilterHandler[index]);
        (void)arm_fir_decimate_q15 (&ARM_Decimator_State[index], (q15_t *)&(PDM_Filter_Out), (q15_t*)&(Decimate_Out), DECIMATOR_BLOCK_SIZE);
        for (ii=0; ii<8U; ii++)
        {
          PCMBuf[(ii * AudioInCtx[Instance].ChannelsNbr) + index] = Decimate_Out[ii];
        }
      }
      else
      {
        (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
      }
    }
  }  
  return ret;
}

/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  pbuf     Main buffer pointer for the recorded data storing  
* @param  NbrOfBytes Not used in this driver - Size depends on PDM clock, defined hard-coded in the BSP
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= (AUDIO_IN_INSTANCES_NBR - 1U) )
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    AudioInCtx[Instance].pBuff = (uint16_t*)pBuf;
    
    if(Instance == 0U)
    {
      UNUSED(NbrOfBytes);
      
      if(AudioInCtx[Instance].ChannelsNbr > 2U)
      {
        if(HAL_SPI_Receive_DMA(&hAudioInSPI, (uint8_t *)SPI_InternalBuffer, (uint16_t)AudioInCtx[Instance].Size) != HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      if(AudioInCtx[Instance].ChannelsNbr != 1U)
      {
        if(AUDIO_IN_Timer_Start() != HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      if(HAL_I2S_Receive_DMA(&hAudioInI2s, I2S_InternalBuffer, (uint16_t)AudioInCtx[Instance].Size/2U) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      } 
      
      /* Update BSP AUDIO IN state */     
      AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
         
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Stop(uint32_t Instance)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    if(Instance == 0U)
    {
      ret = BSP_ERROR_NONE;
      
      if(AudioInCtx[Instance].ChannelsNbr > 2U)
      {
        if(HAL_SPI_DMAStop(&hAudioInSPI)!= HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      if(HAL_I2S_DMAStop(&hAudioInI2s) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      
    }
    else /*(Instance == 1U) */
    { 
      ret =  BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
  } 
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Pause(uint32_t Instance)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {  
    if(Instance == 0U)
    { 
      if(HAL_I2S_DMAPause(&hAudioInI2s)!= HAL_OK)
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else /* (Instance == 1U) */
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_PAUSE;    
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Resume the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Resume(uint32_t Instance)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else 
  {
    if(Instance == 0U)
    {    
      if(HAL_I2S_DMAResume(&hAudioInI2s)!= HAL_OK)
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else /* (Instance == 1U) */
    {
  ret = BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Starts audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  pBuf      Main buffer pointer for the recorded data storing
* @param  size      Size of the recorded buffer
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance != 1U)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    UNUSED(pBuf);
    UNUSED(NbrOfBytes);
    ret = BSP_ERROR_WRONG_PARAM;
  }
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital input device to be stopped
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device)
{
  int32_t ret;
  
  /* Stop selected devices */
  ret = BSP_AUDIO_IN_PauseChannels(Instance, Device);
  
  if(ret == BSP_ERROR_NONE)
  {    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be paused
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }      
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Resume the audio file stream
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be resumed
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN SAI PDM Instance. It can be only 2
* @param  pbuf     Main buffer pointer for the recorded data storing  
* @param  Size     Size of the record buffer
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{
  int32_t ret;
  
  if(Instance != 2U)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    UNUSED(pBuf);
    UNUSED(NbrOfBytes);
    ret = BSP_ERROR_WRONG_PARAM;
  }
  
  /* Return BSP status */
  return ret;
}


/**
* @brief  Set Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device to be used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {  
    if(Instance == 1U)
    {
      ret =  BSP_ERROR_WRONG_PARAM; 
    }
    audio_init.Device = Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume;
    
    if(BSP_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */  
  return ret;
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio Input Device */
    *Device = AudioInCtx[Instance].Device;
  }
  return ret;
}

/**
* @brief  Set Audio In frequency
* @param  Instance     Audio IN instance
* @param  SampleRate  Input frequency to be set
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t  SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(BSP_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }   
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get Audio In frequency
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  SampleRate  Audio Input frequency to be returned
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Return audio in frequency */
    *SampleRate = AudioInCtx[Instance].SampleRate;
  }
  
  /* Return BSP status */  
  return ret;
}

/**
* @brief  Set Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be set
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      ret =  BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(BSP_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */  
  return ret;
}

/**
* @brief  Get Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be returned
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio in resolution */
    *BitsPerSample = AudioInCtx[Instance].BitsPerSample;
  }
  return ret;
}

/**
* @brief  Set Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance >= AUDIO_IN_INSTANCES_NBR) || (ChannelNbr > 2U))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].ChannelsNbr = ChannelNbr;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Get Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Channel number to be returned */
    *ChannelNbr = AudioInCtx[Instance].ChannelsNbr;
  }
  return ret;
}

/**
* @brief  Set the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (Instance == 0U)
  {
    uint32_t index;      
    static int16_t VolumeGain[] = 
    {
      -12,-12,-6,-3,0,2,3,5,6,7,8,9,9,10,11,11,12,12,13,13,14,14,15,15,15,
      16,16,17,17,17,17,18,18,18,19,19,19,19,19,20,20,20,20,21,21,21,21,21,
      22,22,22,22,22,22,23,23,23,23,23,23,23,24,24,24,24,24,24,24,25,25,25,
      25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,
      27,27,28,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,
      29,29,29,29,30,30,30,30,30,30,30,31  
    };
    for (index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
      if (PDM_FilterConfig[index].mic_gain != VolumeGain[Volume])
      {
        PDM_FilterConfig[index].mic_gain = VolumeGain[Volume];
        (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
      }
    }
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].Volume = Volume;
  }
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }  
  else
  {
    /* Input Volume to be returned */
    *Volume = AudioInCtx[Instance].Volume;
  }
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  State     Audio Out state
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Input State to be returned */
    *State = AudioInCtx[Instance].State;
  }
  return ret;
}



/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the BSP_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);
  uint32_t index;
  
  switch(AudioInCtx[0].ChannelsNbr){
  case 1:
    {
      uint16_t * DataTempI2S = &I2S_InternalBuffer[AudioInCtx[0].Size/4U] ;
      for(index = 0; index < (AudioInCtx[0].Size/4U); index++)
      {
        AudioInCtx[0].pBuff[index] = HTONS(DataTempI2S[index]);
      }
      break;
    }
    
  case 2:
    {      
      uint16_t * DataTempI2S = &(I2S_InternalBuffer[AudioInCtx[0].Size/2U]);
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) {
        a = ((uint8_t *)(DataTempI2S))[(index*2U)];
        b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] | (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] | (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
      }
      break;
    }    
  case 4:
    {      
      uint16_t * DataTempI2S = &(I2S_InternalBuffer[AudioInCtx[0].Size/2U]);
      uint16_t * DataTempSPI = &(SPI_InternalBuffer[AudioInCtx[0].Size/2U]);
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) {
        
        a = ((uint8_t *)(DataTempI2S))[(index*2U)];
        b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
          ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
            
            a = ((uint8_t *)(DataTempSPI))[(index*2U)];
            b = ((uint8_t *)(DataTempSPI))[(index*2U)+1U];
            ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+2U] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
              (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
              ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+3U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
                (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
      }
      break;
    }
  default:
    {
      
      break;
    }
    
  }
  
  BSP_AUDIO_IN_TransferComplete_CallBack(0);
}

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the BSP_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);
  uint32_t index;
  switch(AudioInCtx[0].ChannelsNbr){
  case 1:
    {
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      for(index = 0; index < (AudioInCtx[0].Size/4U); index++)
      {
        AudioInCtx[0].pBuff[index] = HTONS(DataTempI2S[index]);
      }
      break;
    }    
  case 2:
    {      
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) {
        a = ((uint8_t *)(DataTempI2S))[(index*2U)];
        b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
          ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
      }      
      break;
    }    
  case 4:
    {      
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      uint16_t * DataTempSPI = SPI_InternalBuffer;
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) 
      {        
        a = ((uint8_t *)(DataTempI2S))[(index*2U)];
        b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
          ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
            
            a = ((uint8_t *)(DataTempSPI))[(index*2U)];
            b = ((uint8_t *)(DataTempSPI))[(index*2U)+1U];
            ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+2U] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
              (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
              ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+3U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
                (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
      }
      break;   
    }
  default:
    {      
      break;
    }
    
  }
  
  BSP_AUDIO_IN_HalfTransfer_CallBack(0);
}


/**
* @brief  User callback when record buffer is filled.
* @retval None
*/
__weak void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @retval None
*/
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Audio IN Error callback function.
* @retval None
*/
__weak void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)
{ 
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function is called when an Interrupt due to transfer error on or peripheral
  error occurs. */
}

/*******************************************************************************
Static Functions
*******************************************************************************/
/**
  * @brief  Register Bus IOs if component ID is OK
  * @retval error status
  */
static int32_t PCM1774_Probe(void)
{
  int32_t ret = BSP_ERROR_NONE;
  PCM1774_IO_t              IOCtx;
  static PCM1774_Object_t   PCM1774Obj;
  
  /* Configure the audio driver */
  IOCtx.BusType     = PCM1774_I2C_BUS;
  IOCtx.Address     = PCM1774_CODEC_I2C_ADDRESS_LOW;
  IOCtx.Init        = BSP_I2C1_Init;
  IOCtx.DeInit      = BSP_I2C1_DeInit;  
  IOCtx.ReadReg     = BSP_I2C1_ReadReg;
  IOCtx.WriteReg    = BSP_I2C1_WriteReg; 
  IOCtx.GetTick     = BSP_GetTick;  
  
  if(PCM1774_RegisterBusIO (&PCM1774Obj, &IOCtx) != PCM1774_OK)
  {
    ret = BSP_ERROR_BUS_FAILURE;   
  }
  else
  {
    AudioDrv = (AUDIO_Drv_t *) (void *) &PCM1774_AUDIO_Driver;
    CompObj = &PCM1774Obj;    
  }
  
  if (AudioDrv->Init(CompObj, (void *)&AudioOutCtx[0]) != PCM1774_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }
  return ret;
} 


/**
  * @brief  Get SAI MX Init configuration
  * @param  hsai SAI handle
  * @param  MXConfig SAI configuration structure
  * @retval None
  */
static void SAI_InitMXConfigStruct(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig)
{
  MXConfig->AudioFrequency    = hsai->Init.AudioFrequency;
  MXConfig->ActiveFrameLength = hsai->FrameInit.ActiveFrameLength;
  MXConfig->AudioMode         = hsai->Init.AudioMode;
  MXConfig->ClockStrobing     = hsai->Init.ClockStrobing;
  MXConfig->DataSize          = hsai->Init.DataSize;
  MXConfig->FrameLength       = hsai->FrameInit.FrameLength;
  MXConfig->MonoStereoMode    = hsai->Init.MonoStereoMode;
  MXConfig->OutputDrive       = hsai->Init.OutputDrive;
  MXConfig->SlotActive        = hsai->SlotInit.SlotActive;
  MXConfig->Synchro           = hsai->Init.Synchro;
  MXConfig->SynchroExt        = hsai->Init.SynchroExt;
  MXConfig->Mckdiv            = hsai->Init.Mckdiv;
  MXConfig->ClockSource       = hsai->Init.ClockSource;
}

/**
  * @brief  Initialize BSP_AUDIO_OUT MSP.
  * @param  hsai  SAI handle 
  * @retval None
  */
static void SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  static DMA_HandleTypeDef hdma_saiTx;
  GPIO_InitTypeDef  GPIO_InitStruct;  
  
  if(hsai->Instance == AUDIO_SAIx)
  {
    /* Enable SAI clock */
    AUDIO_SAIx_CLK_ENABLE();
    
    /* Enable GPIO clock */
    AUDIO_SAIx_SCK_ENABLE();  
    AUDIO_SAIx_SD_ENABLE();  
    AUDIO_SAIx_MCLK_ENABLE(); 
    AUDIO_SAIx_FS_ENABLE();  
    
    /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
    GPIO_InitStruct.Pin = AUDIO_SAIx_MCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = AUDIO_SAIx_MCLK_SCK_SD_FS_AF;
    HAL_GPIO_Init(AUDIO_SAIx_MCLK_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = AUDIO_SAIx_SCK_PIN;
    HAL_GPIO_Init(AUDIO_SAIx_SCK_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = AUDIO_SAIx_FS_PIN;
    HAL_GPIO_Init(AUDIO_SAIx_FS_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = AUDIO_SAIx_SD_PIN;
    HAL_GPIO_Init(AUDIO_SAIx_SD_GPIO_PORT, &GPIO_InitStruct);
    
    /* Enable the DMA clock */
    AUDIO_SAIx_DMAx_CLK_ENABLE();
    
    /* Configure the hdma_saiTx handle parameters */
    hdma_saiTx.Init.Channel             = AUDIO_SAIx_DMAx_CHANNEL;
    hdma_saiTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_saiTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_saiTx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_saiTx.Init.PeriphDataAlignment = AUDIO_SAIx_DMAx_PERIPH_DATA_SIZE;
    hdma_saiTx.Init.MemDataAlignment    = AUDIO_SAIx_DMAx_MEM_DATA_SIZE;
    hdma_saiTx.Init.Mode                = DMA_CIRCULAR;
    hdma_saiTx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_saiTx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_saiTx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_saiTx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_saiTx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    
    hdma_saiTx.Instance = AUDIO_SAIx_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hdma_saiTx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_saiTx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_saiTx);      
    
    /* SAI DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(AUDIO_SAIx_DMAx_IRQ, BSP_AUDIO_OUT_IT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(AUDIO_SAIx_DMAx_IRQ);
  }
}

/**
  * @brief  Deinitializes SAI MSP.
  * @param  hsai  SAI handle 
  * @retval HAL status
  */
static void SAI_MspDeInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef  gpio_init_structure;
  if(hsai->Instance == AUDIO_SAIx)
  {  
    /* SAI DMA IRQ Channel deactivation */
    HAL_NVIC_DisableIRQ(AUDIO_SAIx_DMAx_IRQ);
    
    /* Deinitialize the DMA stream */
    (void)HAL_DMA_DeInit(hsai->hdmatx);
    
    /* Disable SAI peripheral */
    __HAL_SAI_DISABLE(hsai);  
    
    /* Deactivates CODEC_SAI pins FS, SCK, MCK and SD by putting them in input mode */
    gpio_init_structure.Pin = AUDIO_SAIx_FS_PIN;
    HAL_GPIO_DeInit(AUDIO_SAIx_FS_GPIO_PORT, gpio_init_structure.Pin);
    
    gpio_init_structure.Pin = AUDIO_SAIx_SCK_PIN;
    HAL_GPIO_DeInit(AUDIO_SAIx_SCK_GPIO_PORT, gpio_init_structure.Pin);
    
    gpio_init_structure.Pin =  AUDIO_SAIx_SD_PIN;
    HAL_GPIO_DeInit(AUDIO_SAIx_SD_GPIO_PORT, gpio_init_structure.Pin);
    
    gpio_init_structure.Pin = AUDIO_SAIx_MCLK_PIN;
    HAL_GPIO_DeInit(AUDIO_SAIx_MCLK_GPIO_PORT, gpio_init_structure.Pin);
    
    /* Disable SAI clock */
    AUDIO_SAIx_CLK_DISABLE();
  }
}


static void I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);	
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the I2S2 peripheral clock */
  AUDIO_IN_I2S_CLK_ENABLE();
  
  /* Enable I2S GPIO clocks */
  AUDIO_IN_I2S_MOSI_GPIO_CLK_ENABLE();
  
  /* I2S2 pins configuration: SCK and MOSI pins ------------------------------*/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  
  if (AudioInCtx[0].ChannelsNbr == 1)
  {
    AUDIO_IN_1CH_I2S_SCK_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin       = AUDIO_IN_1CH_I2S_SCK_PIN;    
    GPIO_InitStruct.Alternate = AUDIO_IN_I2S_SCK_AF;
    HAL_GPIO_Init(AUDIO_IN_1CH_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
  }
  else
  {      
    AUDIO_IN_I2S_SCK_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin       = AUDIO_IN_I2S_SCK_PIN;
    GPIO_InitStruct.Alternate = AUDIO_IN_I2S_SCK_AF;
    HAL_GPIO_Init(AUDIO_IN_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
  }
  
  GPIO_InitStruct.Pin       = AUDIO_IN_I2S_MOSI_PIN ;
  GPIO_InitStruct.Alternate = AUDIO_IN_I2S_MOSI_AF;
  HAL_GPIO_Init(AUDIO_IN_I2S_MOSI_GPIO_PORT, &GPIO_InitStruct);
 
} 

static void SPI_MspInit(SPI_HandleTypeDef *hspi)
{  
  UNUSED(hspi);
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable GPIO TX/RX clock */
  AUDIO_IN_SPI_SCK_GPIO_CLK_ENABLE();
  AUDIO_IN_SPI_MISO_GPIO_CLK_ENABLE();
  AUDIO_IN_SPI_MOSI_GPIO_CLK_ENABLE();
  /* Enable SPI3 clock */
  AUDIO_IN_SPI_CLK_ENABLE();
  /* Enable DMA1 clock */
  AUDIO_IN_SPI_DMAx_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/
  /* SPI SCK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = AUDIO_IN_SPI_SCK_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = AUDIO_IN_SPI_SCK_AF;
  
  HAL_GPIO_Init(AUDIO_IN_SPI_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  /* SPI MOSI GPIO pin configuration  */
  GPIO_InitStruct.Pin = AUDIO_IN_SPI_MOSI_PIN;
  GPIO_InitStruct.Alternate = AUDIO_IN_SPI_MOSI_AF;
  HAL_GPIO_Init(AUDIO_IN_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);
  
  
}


/**
* @brief Audio Timer Init
* @param None
* @retval None
*/
static HAL_StatusTypeDef AUDIO_IN_Timer_Init(void)
{
  HAL_StatusTypeDef ret =  HAL_OK;
  static TIM_SlaveConfigTypeDef   sSlaveConfig;
  static TIM_IC_InitTypeDef       sICConfig;
  static TIM_OC_InitTypeDef       sOCConfig; 
  GPIO_InitTypeDef   GPIO_InitStruct;
  
  /* Enable AUDIO_TIMER clock*/
  AUDIO_IN_TIMER_CLK_ENABLE();
  AUDIO_IN_TIMER_CHOUT_GPIO_PORT_CLK_ENABLE();
  AUDIO_IN_TIMER_CHIN_GPIO_PORT_CLK_ENABLE();
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHIN_AF;
  GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHIN_PIN;
  HAL_GPIO_Init(AUDIO_IN_TIMER_CHIN_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHOUT_AF;
  GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHOUT_PIN;
  HAL_GPIO_Init(AUDIO_IN_TIMER_CHOUT_GPIO_PORT, &GPIO_InitStruct);
  
  TimDividerHandle.Instance = AUDIO_IN_TIMER;
  
  /* Configure the Input: channel_1 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter = 0;
  if(HAL_TIM_IC_ConfigChannel(&TimDividerHandle, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  
  /* Configure TIM1 in Gated Slave mode for the external trigger (Filtered Timer
  Input 1) */
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_EXTERNAL1;
  if( HAL_TIM_SlaveConfigSynchronization(&TimDividerHandle, &sSlaveConfig) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  
  /* Initialize TIM3 peripheral in PWM mode*/
  TimDividerHandle.Init.Period            = 1;
  TimDividerHandle.Init.Prescaler         = 0;
  TimDividerHandle.Init.ClockDivision     = 0;
  TimDividerHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimDividerHandle.Init.RepetitionCounter = 0;
  if(HAL_TIM_PWM_Init(&TimDividerHandle) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  
  /* Configure the PWM_channel_1  */
  sOCConfig.OCMode     = TIM_OCMODE_PWM1;
  sOCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOCConfig.Pulse = 1;
  if(HAL_TIM_PWM_ConfigChannel(&TimDividerHandle, &sOCConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  return ret;
}

/**
* @brief Audio Timer Start
* @param None
* @retval None
*/
static HAL_StatusTypeDef AUDIO_IN_Timer_Start(void)
{
  
  HAL_StatusTypeDef ret =  HAL_OK;
  if(HAL_TIM_IC_Start(&TimDividerHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  /* Start the Output Compare */
  if(HAL_TIM_OC_Start(&TimDividerHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  
  return ret;
}

/**
* @}
*/ 

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/ 
