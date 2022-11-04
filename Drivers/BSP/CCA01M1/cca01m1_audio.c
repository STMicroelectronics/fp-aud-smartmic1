/**
  ******************************************************************************
  * @file    cca01m1_audio.c
  * @author  SRA
  * @brief   This file provides a set of functions needed to manage the 
  *          sound terminal device.
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
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "cca01m1_audio.h"
#include "audio.h"

/** @addtogroup BSP BSP
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1 X_NUCLEO_CCA01M1
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1_AUDIO X_NUCLEO_CCA01M1_AUDIO
* @{
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_Defines Private Defines
* @{
*/
#ifndef NULL
#define NULL      (void *) 0
#endif
  
#ifdef USE_STM32L4XX_NUCLEO
#define SAI_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (12U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (2U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (6U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (1U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (3U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (0U) \
         : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (2U) : (1U)
#endif
           
/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_Variables Private Variables
* @{
*/
AUDIO_OUT_Ctx_t                 AudioOutCtx[AUDIO_OUT_INSTANCES_NBR] = {0};

static AUDIO_Drv_t              *AudioDrv = NULL;
void                            *CompObj = NULL;

#ifdef USE_STM32L4XX_NUCLEO
SAI_HandleTypeDef hAudioOut[SOUNDTERMINAL_DEVICE_NBR];
#else
I2S_HandleTypeDef hAudioOut[SOUNDTERMINAL_DEVICE_NBR];
#endif
/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_Function_Prototypes Private Function Prototypes
* @{
*/
#ifdef USE_STM32L4XX_NUCLEO
/* SAI Msp config */
static void SAI_MspInit(SAI_HandleTypeDef *hsai);
static void SAI_MspDeInit(SAI_HandleTypeDef *hsai);
static void SAI_InitMXConfigStruct(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig);
#else
/* I2S Msp config */
static void I2S_MspInit(I2S_HandleTypeDef *hi2s);
static void I2S_MspDeInit(I2S_HandleTypeDef *hi2s);
static void I2S_InitMXConfigStruct(I2S_HandleTypeDef* hi2s, MX_I2S_OUT_Config *MXConfig);
#endif

static int32_t STA350BW_Probe(void);

/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Exported_Functions Exported Functions
* @{
*/

/**
* @brief  Initialization of AUDIO Device.
* @param  handle: device handle 
* @param  Volume: initialization volume
* @param  AudioFreq: sampling frequency
* @retval COMPONENT_OK if no problem during initialization, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Init(uint32_t Instance, CCA01M1_AUDIO_Init_t* AudioInit)
{ 
  uint8_t tmp;
  
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
    
    if (Instance == 0U) /* Use SAI peripheral: configuration valid only with NUCLEO-L476RG */
    { 
#ifndef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      switch(AudioOutCtx[Instance].Device)
      {
      case CODEC_SENSORS_AUTO:
      default:
        {
          tmp = STA350BW_0;  
          hAudioOut[STA350BW_0].Instance = AUDIO_OUT1_SAI_INSTANCE;   
          break;
        }
      case STA350BW_0:
        {
          tmp = STA350BW_0;
          hAudioOut[STA350BW_0].Instance = AUDIO_OUT1_SAI_INSTANCE;   
          break;
        }
      case STA350BW_1:
        {
          tmp = STA350BW_1;
          hAudioOut[STA350BW_1].Instance = AUDIO_OUT2_SAI_INSTANCE;
          break;
        }
      }   
      
      /* PLL clock SETUP */ 
      if(MX_SAI_ClockConfig(&hAudioOut[tmp], AudioOutCtx[Instance].SampleRate) != HAL_OK)
      {
        return BSP_ERROR_CLOCK_FAILURE;
      }
      
      if(HAL_SAI_GetState(&hAudioOut[tmp]) == HAL_SAI_STATE_RESET)
      {
        /* Init the SAI MSP */
        SAI_MspInit(&hAudioOut[tmp]);
      }   
      
      MX_SAI_Config mx_sai_config;
      
      /* Prepare hAudioOutSai handle */
      mx_sai_config.Mckdiv            = SAI_CLOCK_DIVIDER(AudioInit->SampleRate);
      mx_sai_config.AudioFrequency    = SAI_AUDIO_FREQUENCY_MCKDIV;
      mx_sai_config.AudioMode         = SAI_MODEMASTER_TX;
      mx_sai_config.ClockStrobing     = SAI_CLOCKSTROBING_RISINGEDGE;
      mx_sai_config.MonoStereoMode    = SAI_STEREOMODE;
      mx_sai_config.DataSize          = SAI_DATASIZE_16;
      mx_sai_config.FrameLength       = 32; 
      mx_sai_config.ActiveFrameLength = 16;  
      mx_sai_config.OutputDrive       = SAI_OUTPUTDRIVE_ENABLE;
      mx_sai_config.Synchro           = SAI_ASYNCHRONOUS;
      mx_sai_config.SlotActive         = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;
      
      /* SAI peripheral initialization */
      if(MX_SAI_Init(&hAudioOut[tmp], &mx_sai_config) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }
      
      uint16_t dummy[16]={0};  
      HAL_SAI_Transmit_DMA(&hAudioOut[tmp], (uint8_t *)dummy, 32);
      
      CCA01M1_AUDIO_OUT_Reset(Instance);
      
      if(STA350BW_Probe()!= BSP_ERROR_NONE)
      {
        return BSP_ERROR_COMPONENT_FAILURE;
      } 
      
      HAL_SAI_DMAStop(&hAudioOut[tmp]);
      
      if(tmp == STA350BW_0)
      {    
        HAL_NVIC_EnableIRQ(AUDIO_OUT1_SAI_DMAx_IRQ);
      }
      else if (tmp == STA350BW_1)
      {
        HAL_NVIC_EnableIRQ(AUDIO_OUT2_SAI_DMAx_IRQ);
      }  
#endif      
    }    
    else if (Instance == 1U)
    {  
#ifdef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else    
      switch(AudioOutCtx[Instance].Device)
      {
      case CODEC_SENSORS_AUTO:
      default:
        {
          tmp = STA350BW_0;  
          hAudioOut[STA350BW_0].Instance = AUDIO_OUT1_I2S_INSTANCE;   
          break;
        }
      case STA350BW_0:
        {
          tmp = STA350BW_0;
          hAudioOut[STA350BW_0].Instance = AUDIO_OUT1_I2S_INSTANCE;   
          break;
        }
#ifndef USE_STM32L0XX_NUCLEO
      case STA350BW_1:
        {
          tmp = STA350BW_1;
          hAudioOut[STA350BW_1].Instance = AUDIO_OUT2_I2S_INSTANCE;
          break;
        }
#endif
      }  
      /* PLL clock SETUP */ 
      if(MX_I2S_OUT_ClockConfig(&hAudioOut[tmp], AudioOutCtx[Instance].SampleRate) != HAL_OK)
      {
        return BSP_ERROR_CLOCK_FAILURE;
      }
      
      if(HAL_I2S_GetState(&hAudioOut[tmp]) == HAL_I2S_STATE_RESET)
      {
        /* Init the I2S MSP */
        I2S_MspInit(&hAudioOut[tmp]);
      }   
      
      MX_I2S_OUT_Config mx_i2s_config;
      
      /* Prepare hAudioOut handle */
      mx_i2s_config.AudioFreq    = AudioOutCtx[Instance].SampleRate;
      mx_i2s_config.CPOL         = I2S_CPOL_LOW;
      mx_i2s_config.DataFormat   = I2S_DATAFORMAT_16B;
      mx_i2s_config.MCLKOutput   = I2S_MCLKOUTPUT_ENABLE;
      mx_i2s_config.Mode         = I2S_MODE_MASTER_TX;
      mx_i2s_config.Standard     = I2S_STANDARD_PHILIPS;
      
#if (!defined(USE_STM32F0XX_NUCLEO) && !defined(USE_STM32L0XX_NUCLEO))
      mx_i2s_config.ClockSource  = I2S_CLOCK_PLL;
#endif
      
      /* SAI peripheral initialization */
      if(MX_I2S_OUT_Init(&hAudioOut[tmp], &mx_i2s_config) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }
      
      uint16_t dummy[16]={0};  
      HAL_I2S_Transmit_DMA(&hAudioOut[tmp], dummy, 16);
      
      CCA01M1_AUDIO_OUT_Reset(Instance);
      
      if(STA350BW_Probe()!= BSP_ERROR_NONE)
      {
        return BSP_ERROR_COMPONENT_FAILURE;
      } 
      
      HAL_I2S_DMAStop(&hAudioOut[tmp]);
      
      if(tmp == STA350BW_0)
      {    
        HAL_NVIC_EnableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ);
      }
#ifndef USE_STM32L0XX_NUCLEO    
      else if (tmp == STA350BW_1)
      {
        HAL_NVIC_EnableIRQ(AUDIO_OUT2_I2S_DMAx_IRQ);
      }  
#endif
#endif
    }
    
    /* Update BSP AUDIO OUT state */
    AudioOutCtx[Instance].State = AUDIO_OUT_STATE_STOP;
  }
  
  return BSP_ERROR_NONE; 
}

/**
* @brief  De-initialization of AUDIO Device.
* @param  handle: device handle 
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_DeInit(uint32_t Instance)
{  
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM; 
  }
  else
  {
    if (Instance == 0U)
    {       
#ifndef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      HAL_SAI_DMAStop(&hAudioOut[tmp]);
      SAI_MspDeInit(&hAudioOut[tmp]);
      /* Call the Media layer stop function */
      if(AudioDrv->DeInit(CompObj) != 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }   
      else if(HAL_SAI_DeInit(&hAudioOut[tmp]) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }   
      else
      {
        /* Update BSP AUDIO OUT state */    
        AudioOutCtx[Instance].State = AUDIO_OUT_STATE_RESET;    
      }
#endif
    }
    else if (Instance == 1U)
    { 
#ifdef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      HAL_I2S_DMAStop(&hAudioOut[tmp]);
      I2S_MspDeInit(&hAudioOut[tmp]);
      /* Call the Media layer stop function */
      if(AudioDrv->DeInit(CompObj) != 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }   
      else if(HAL_I2S_DeInit(&hAudioOut[tmp]) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }   
      else
      {
        /* Update BSP AUDIO OUT state */    
        AudioOutCtx[Instance].State = AUDIO_OUT_STATE_RESET;    
      }
#endif
    }
  }
  /* Return BSP status */
  return ret; 
}


/**
* @brief  Starts audio streaming to the AUDIO Device.
* @param  handle: device handle
* @param  *pBuffer: pointer to the data to be streamed
* @param  Size: data size
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Play(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if((Instance >= AUDIO_OUT_INSTANCES_NBR) || (((NbrOfBytes / (AudioOutCtx[Instance].BitsPerSample/8U)) > 0xFFFFU)))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }  
  else if((AudioOutCtx[Instance].State == AUDIO_OUT_STATE_STOP) || (AudioOutCtx[Instance].State == AUDIO_OUT_STATE_RESET))
  {      
    if (Instance == 0U)
    { 
#ifndef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      if (HAL_SAI_Transmit_DMA(&hAudioOut[tmp], (uint8_t *)pData, DMA_MAX(NbrOfBytes))!= HAL_OK) 
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      } 
#endif
    }
    else if (Instance == 1U)
    {
#ifdef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      if (HAL_I2S_Transmit_DMA(&hAudioOut[tmp], (uint16_t *)pData, DMA_MAX(NbrOfBytes))!= HAL_OK) 
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif
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
* @brief  This function Pauses the audio stream. In case
*         of using DMA, the DMA Pause feature is used.
* @param  handle: device handle
* @WARNING When calling CCA01M1_AUDIO_OUT_Pause() function for pause, only
*          CCA01M1_AUDIO_OUT_Resume() function should be called for resume (use of CCA01M1_AUDIO_OUT_Play() 
*          function for resume could lead to unexpected behavior).
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Pause(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  { 
    /* Call the Audio Codec Pause/Resume function */
    if(AudioDrv->Pause(CompObj) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    
    if (Instance == 0U)
    { 
#ifndef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      if (HAL_SAI_DMAPause(&hAudioOut[tmp])!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif
    }
    else if (Instance == 1U)
    {
#ifdef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else    
      if (HAL_I2S_DMAPause(&hAudioOut[tmp])!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }   
#endif 
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
* @brief  Resumes the audio stream.
* @param  handle: device handle
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Resume(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Call the Audio Codec Pause/Resume function */
    if(AudioDrv->Resume(CompObj) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
        
    if (Instance == 0U)
    { 
#ifndef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      if (HAL_SAI_DMAResume(&hAudioOut[tmp])!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif       
    }
    else if (Instance == 1U)
    {  
#ifdef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else  
      if (HAL_I2S_DMAResume(&hAudioOut[tmp])!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif    
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
* @brief  Stop the audio stream.
* @param  handle: device handle
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Stop(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (AudioOutCtx[Instance].State == AUDIO_OUT_STATE_PLAYING)
  { 
    if (Instance == 0U)
    { 
#ifndef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      if (HAL_SAI_DMAStop(&hAudioOut[tmp])!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif       
    }
    else if (Instance == 1U)
    { 
#ifdef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else   
      if (HAL_I2S_DMAStop(&hAudioOut[tmp])!= HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif    
    }
    
    if( ret==BSP_ERROR_NONE)
    {
      /* Update BSP AUDIO OUT state */    
      AudioOutCtx[Instance].State = AUDIO_OUT_STATE_STOP;  
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
* @brief  Set volume.
* @param  handle: device handle
* @param  channel: channel to be configured
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  int32_t ii;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  { 
    for (ii = 0; ii<AudioOutCtx[Instance].ChannelsNbr; ii++)
    {
      if(AudioDrv->SetVolume(CompObj, ii, Volume) != 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else if(Volume == 0U)
      {
        /* Update Mute State */
        AudioOutCtx[Instance].IsMute = CCA01M1_AUDIO_MUTE_ENABLED;
      }
      else
      {
        /* Update Mute State */
        AudioOutCtx[Instance].IsMute = CCA01M1_AUDIO_MUTE_DISABLED;      
      }
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
int32_t CCA01M1_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume)
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
* @brief  Set mute.
* @param  handle: device handle
* @param  channel: channel to be muted
* @param  state: enable or disable value
*         This parameter can be a value of @ref STA350BW_state_define 
* @param  filterValues: pointer to filter values    
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_Mute(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  { 
    ret = BSP_ERROR_WRONG_PARAM;    
  }
  else
  {
    if(AudioDrv->SetMute(CompObj, CODEC_MUTE_ON) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = CCA01M1_AUDIO_MUTE_ENABLED;
    }
  }
  /* Return BSP status */
  return ret;    
}


/**
* @brief  Set unmute.
* @param  handle: device handle
* @param  channel: channel to be muted
* @param  state: enable or disable value
*         This parameter can be a value of @ref STA350BW_state_define 
* @param  filterValues: pointer to filter values    
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_UnMute(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR) 
  { 
    ret = BSP_ERROR_WRONG_PARAM;    
  }
  else
  {
    if(AudioDrv->SetMute(CompObj, CODEC_MUTE_OFF) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      /* Update Mute State */
      AudioOutCtx[Instance].IsMute = CCA01M1_AUDIO_MUTE_DISABLED;
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
int32_t CCA01M1_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute)
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
int32_t CCA01M1_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
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
    
    if (Instance == 0U)
    { 
#ifndef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      MX_SAI_Config mx_out_config;
      /* Get SAI MX configuration */
      SAI_InitMXConfigStruct(&hAudioOut[Device], &mx_out_config);
      if(MX_SAI_Init(&hAudioOut[Device], &mx_out_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif      
    }
    else if (Instance == 1U)
    {
#ifdef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      MX_I2S_OUT_Config mx_out_config;
      /* Get SAI MX configuration */
      I2S_InitMXConfigStruct(&hAudioOut[Device], &mx_out_config);
      if(MX_I2S_OUT_Init(&hAudioOut[Device], &mx_out_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif
    }
    /* Update AudioOutCtx structure */
    AudioOutCtx[Instance].Device = Device;
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
int32_t CCA01M1_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device)
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
* @brief  Set frequency of the I2S bus.
* @param  handle: device handle
* @param  AudioFreq: sampling frequency
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tmp = AudioOutCtx[Instance].Device;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    if(AudioDrv->SetFrequency(CompObj, SampleRate) != 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }  
    
    if (Instance == 0U)
    { 
#ifndef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      MX_SAI_Config mx_sai_config;
      
      /* Get I2S MX configuration */
      SAI_InitMXConfigStruct(&hAudioOut[tmp], &mx_sai_config);
      /* PLL clock setup */ 
      
      if(MX_SAI_ClockConfig(&hAudioOut[tmp], SampleRate) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else if(MX_SAI_Init(&hAudioOut[tmp], &mx_sai_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif      
    }
    else if (Instance == 1U)
    {
#ifdef USE_STM32L4XX_NUCLEO
      return BSP_ERROR_WRONG_PARAM; 
#else
      MX_I2S_OUT_Config mx_i2s_config;
      
      /* Get I2S MX configuration */
      I2S_InitMXConfigStruct(&hAudioOut[tmp], &mx_i2s_config);
      /* PLL clock setup */ 
      
      if(MX_I2S_OUT_ClockConfig(&hAudioOut[tmp], SampleRate) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else if(MX_I2S_OUT_Init(&hAudioOut[tmp], &mx_i2s_config) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
#endif
    }
    /* Store new sample rate */
    AudioOutCtx[Instance].SampleRate = SampleRate;     
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
int32_t CCA01M1_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
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

int32_t CCA01M1_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioOutCtx[Instance].State != AUDIO_OUT_STATE_PLAYING)
  {
    AudioOutCtx[Instance].BitsPerSample = BitsPerSample;
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
int32_t CCA01M1_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
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
int32_t CCA01M1_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
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
int32_t CCA01M1_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
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
int32_t CCA01M1_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State)
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
* @brief  Reset the device.
* @param  handle: device handle
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise 
* @retval None
*/
int32_t CCA01M1_AUDIO_OUT_Reset(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_OUT_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {    
    if(AudioOutCtx[Instance].Device == STA350BW_0)
    {
      HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_RESET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_SET);  
    }
#ifndef USE_STM32L0XX_NUCLEO
    else if (AudioOutCtx[Instance].Device == STA350BW_1)
    {
      HAL_GPIO_WritePin(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN, GPIO_PIN_RESET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN, GPIO_PIN_SET);   
    }
#endif
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }
  return ret; 
}

/**
* @brief  Manages the DMA full Transfer complete event.
* @param  OutputDevice: device relevant with the thrown interrupt
*         This parameter can be a value of @ref CODEC_ID_t 
* @retval None
*/
__weak void CCA01M1_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance); 
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @param  OutputDevice: device relevant with the thrown interrupt
*         This parameter can be a value of @ref CODEC_ID_t 
* @retval None
*/
__weak void CCA01M1_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance); 
}



/*******************************************************************************
Static Functions
*******************************************************************************/
/**
* @brief  Register Bus IOs if component ID is OK
* @retval error status
*/
static int32_t STA350BW_Probe(void)
{
  int32_t ret = BSP_ERROR_NONE;
  STA350BW_IO_t              IOCtx;
  static STA350BW_Object_t   STA350BWObj;
  
  /* Configure the audio driver */
  IOCtx.BusType     = STA350BW_I2C_BUS;
  IOCtx.Init        = BSP_I2C1_Init;
  IOCtx.DeInit      = BSP_I2C1_DeInit;  
  IOCtx.ReadReg     = BSP_I2C1_ReadReg;
  IOCtx.WriteReg    = BSP_I2C1_WriteReg; 
  IOCtx.GetTick     = BSP_GetTick;  
  
  if (AudioOutCtx[0].Device == STA350BW_0 && AudioOutCtx[0].ChannelsNbr != 0)
  {
    IOCtx.Address     = STA350BW_ADDRESS_1;
  }
  else if (AudioOutCtx[0].Device == STA350BW_1 && AudioOutCtx[0].ChannelsNbr != 0)
  {
    IOCtx.Address     = STA350BW_ADDRESS_2;
  }
  else if (AudioOutCtx[1].Device == STA350BW_0 && AudioOutCtx[1].ChannelsNbr != 0)
  {
    IOCtx.Address     = STA350BW_ADDRESS_1;
  }
#ifndef USE_STM32L0XX_NUCLEO  
  else if (AudioOutCtx[1].Device == STA350BW_1 && AudioOutCtx[1].ChannelsNbr != 0)
  {
    IOCtx.Address     = STA350BW_ADDRESS_2;
  }
#endif
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  
  if(STA350BW_RegisterBusIO (&STA350BWObj, &IOCtx) != STA350BW_OK)
  {
    ret = BSP_ERROR_BUS_FAILURE;   
  }
  else
  {
    AudioDrv = (AUDIO_Drv_t *) (void *) &STA350BW_AUDIO_Driver;
    CompObj = &STA350BWObj;    
  }
  
#ifdef USE_STM32L4XX_NUCLEO
  if (AudioDrv->Init(CompObj, (void *)&AudioOutCtx[0]) != STA350BW_OK)
#else
    if (AudioDrv->Init(CompObj, (void *)&AudioOutCtx[1]) != STA350BW_OK)
#endif    
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  return ret;
} 


#ifdef USE_STM32L4XX_NUCLEO
/**
  * @brief  SAI clock Config.
  * @param  hsai SAI handle
  * @param  SampleRate  Audio frequency used to play the audio stream.
  * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
  *         Being __weak it can be overwritten by the application     
  * @retval HAL status
  */
__weak HAL_StatusTypeDef MX_SAI_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t SampleRate)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);
  
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  
  if (!(SampleRate % 8))
  {
    /* Audio frequency multiple of 8 (8/16/32/48/96/192)*/    
    RCC_PeriphCLKInitStruct.PLLSAI2.PLLSAI2N        = 43;
    RCC_PeriphCLKInitStruct.PLLSAI2.PLLSAI2P        = 7;    
  }
  else
  {
    RCC_PeriphCLKInitStruct.PLLSAI2.PLLSAI2N        = 24;
    RCC_PeriphCLKInitStruct.PLLSAI2.PLLSAI2P        = 17;
  }
  
  RCC_PeriphCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI2;
  RCC_PeriphCLKInitStruct.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_SAI2CLK;
  RCC_PeriphCLKInitStruct.Sai2ClockSelection      = RCC_SAI2CLKSOURCE_PLLSAI2;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  
  return ret;
}

/**
  * @brief  Initializes the Audio Codec audio out instance (SAI).
  * @param  MXConfig SAI configuration structure
  * @note   Being __weak it can be overwritten by the application
  * @retval HAL status
  */
__weak HAL_StatusTypeDef MX_SAI_Init(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig)
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
  hsai->Init.Mckdiv               = MXConfig->Mckdiv;
  
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
* @brief Tx Transfer completed callbacks
* @param hSai: I2S handle
* @retval None
*/
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hSai)
{
  /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
  if(hSai->Instance == AUDIO_OUT1_SAI_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_TransferComplete_CallBack(0);
  }
  if(hSai->Instance == AUDIO_OUT2_SAI_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_TransferComplete_CallBack(0);
  }    
}

/**
* @brief Tx Transfer Half completed callbacks
* @param hSai: I2S handle
* @retval None
*/
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hSai)
{
  /* Manage the remaining file size and new address offset: This function 
  should be coded by user (its prototype is already declared in stm324xg_eval_audio.h) */  
  if(hSai->Instance == AUDIO_OUT1_SAI_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_HalfTransfer_CallBack(0);
  }
  if(hSai->Instance == AUDIO_OUT2_SAI_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_HalfTransfer_CallBack(0);
  }
}


/**
* @brief  Get I2S MX Init configuration
* @param  hi2s I2S handle
* @param  MXConfig I2S configuration structure
* @retval None
*/
static void SAI_InitMXConfigStruct(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig)
{
  /* Configure SAI1 */
  MXConfig->MonoStereoMode     = hsai->Init.MonoStereoMode       ;
  MXConfig->AudioFrequency     = hsai->Init.AudioFrequency       ;
  MXConfig->AudioMode         = hsai->Init.AudioMode            ;
  MXConfig->DataSize         = hsai->Init.DataSize             ;
  MXConfig->ClockStrobing     = hsai->Init.ClockStrobing        ;
  MXConfig->Synchro           = hsai->Init.Synchro              ;
  MXConfig->OutputDrive       = hsai->Init.OutputDrive          ;
  MXConfig->Mckdiv           = hsai->Init.Mckdiv               ;
  MXConfig->FrameLength       = hsai->FrameInit.FrameLength     ;
  MXConfig->ActiveFrameLength = hsai->FrameInit.ActiveFrameLength;
  MXConfig->SlotActive        = hsai->SlotInit.SlotActive       ;
}

/**
* @brief  Initialize CCA01M1_AUDIO_OUT MSP.
* @param  hi2s  I2S handle 
* @retval None
*/
static void SAI_MspInit(SAI_HandleTypeDef *hSai)
{
  GPIO_InitTypeDef  gpio_init_structure;  
  
  if(AudioOutCtx[0].Device == STA350BW_0)
  {
    static DMA_HandleTypeDef hdma_SaiTx_1;   
    
    AUDIO_OUT1_SAI_CLK_ENABLE();
    AUDIO_OUT1_SAI_SD_CLK_ENABLE(); 
    AUDIO_OUT1_SAI_SCK_CLK_ENABLE();
    AUDIO_OUT1_SAI_MCK_CLK_ENABLE();
    AUDIO_OUT1_SAI_FS_CLK_ENABLE();
    
    /* I2S3 pins configuration: WS, SCK and SD pins -----------------------------*/
    gpio_init_structure.Pin         = AUDIO_OUT1_SAI_SCK_PIN ; 
    gpio_init_structure.Mode        = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull        = GPIO_PULLDOWN;
    gpio_init_structure.Speed       = GPIO_SPEED_FAST;
    gpio_init_structure.Alternate   = AUDIO_OUT1_SAI_SCK_SD_WS_MCK_AF;
    HAL_GPIO_Init(AUDIO_OUT1_SAI_SCK_GPIO_PORT, &gpio_init_structure);
    
    gpio_init_structure.Pin         = AUDIO_OUT1_SAI_SD_PIN ;
    HAL_GPIO_Init(AUDIO_OUT1_SAI_SD_GPIO_PORT, &gpio_init_structure);     
    
    gpio_init_structure.Pin         = AUDIO_OUT1_SAI_FS_PIN ;
    HAL_GPIO_Init(AUDIO_OUT1_SAI_FS_GPIO_PORT, &gpio_init_structure); 
    
    gpio_init_structure.Pin         = AUDIO_OUT1_SAI_MCK_PIN; 
    HAL_GPIO_Init(AUDIO_OUT1_SAI_MCK_GPIO_PORT, &gpio_init_structure);   
    
    /* Enable the I2S DMA clock */
    AUDIO_OUT1_SAI_DMAx_CLK_ENABLE(); 
    
    if(hSai->Instance == AUDIO_OUT1_SAI_INSTANCE)
    {
      /* Configure the hdma_i2sTx handle parameters */  
      
      hdma_SaiTx_1.Instance             = AUDIO_OUT1_SAI_DMAx_INSTANCE;  
      hdma_SaiTx_1.Init.Request             = AUDIO_OUT1_SAI_DMAx_REQUEST;  
      hdma_SaiTx_1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_SaiTx_1.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_SaiTx_1.Init.MemInc              = DMA_MINC_ENABLE;
      hdma_SaiTx_1.Init.PeriphDataAlignment = AUDIO_OUT1_SAI_DMAx_PERIPH_DATA_SIZE;
      hdma_SaiTx_1.Init.MemDataAlignment    = AUDIO_OUT1_SAI_DMAx_MEM_DATA_SIZE;
      hdma_SaiTx_1.Init.Mode                = DMA_CIRCULAR;
      hdma_SaiTx_1.Init.Priority            = DMA_PRIORITY_HIGH;      
      
      /* Associate the DMA handle */
      __HAL_LINKDMA(hSai, hdmatx, hdma_SaiTx_1);
      
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(&hdma_SaiTx_1);
      
      /* Configure the DMA Stream */
      HAL_DMA_Init(&hdma_SaiTx_1);
      
      HAL_NVIC_DisableIRQ(AUDIO_OUT1_SAI_DMAx_IRQ); 
    }
    
    /*Reset pin configuration*/
    AUDIO_OUT1_RST_GPIO_CLK_ENABLE();
    
    /* Configure GPIO PINs to detect Interrupts */
    gpio_init_structure.Pin = AUDIO_OUT1_RST_PIN;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(AUDIO_OUT1_RST_GPIO_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_SET);   
    
    HAL_NVIC_SetPriority(AUDIO_OUT1_SAI_DMAx_IRQ, CCA01M1_AUDIO_OUT_IT_PRIORITY, 0);
    
  }
  else if (AudioOutCtx[0].Device == STA350BW_1)
  {   
    static DMA_HandleTypeDef hdma_SaiTx_2;
    
    AUDIO_OUT2_SAI_CLK_ENABLE();
    AUDIO_OUT2_SAI_SD_CLK_ENABLE(); 
    AUDIO_OUT2_SAI_SCK_CLK_ENABLE();
    AUDIO_OUT2_SAI_MCK_CLK_ENABLE();
    AUDIO_OUT2_SAI_FS_CLK_ENABLE();
    
    /* I2S3 pins configuration: WS, SCK and SD pins -----------------------------*/
    gpio_init_structure.Pin         = AUDIO_OUT2_SAI_SCK_PIN ; 
    gpio_init_structure.Mode        = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull        = GPIO_PULLDOWN;
    gpio_init_structure.Speed       = GPIO_SPEED_FAST;
    gpio_init_structure.Alternate   = AUDIO_OUT2_SAI_SCK_SD_WS_MCK_AF;
    HAL_GPIO_Init(AUDIO_OUT2_SAI_SCK_GPIO_PORT, &gpio_init_structure);
    
    gpio_init_structure.Pin         = AUDIO_OUT2_SAI_SD_PIN ;
    HAL_GPIO_Init(AUDIO_OUT2_SAI_SD_GPIO_PORT, &gpio_init_structure);     
    
    gpio_init_structure.Pin         = AUDIO_OUT2_SAI_FS_PIN ;
    HAL_GPIO_Init(AUDIO_OUT2_SAI_FS_GPIO_PORT, &gpio_init_structure); 
    
    gpio_init_structure.Pin         = AUDIO_OUT2_SAI_MCK_PIN; 
    HAL_GPIO_Init(AUDIO_OUT2_SAI_MCK_GPIO_PORT, &gpio_init_structure);   
    
    /* Enable the I2S DMA clock */
    AUDIO_OUT2_SAI_DMAx_CLK_ENABLE(); 
    
    if(hSai->Instance == AUDIO_OUT2_SAI_INSTANCE)
    {
      /* Configure the hdma_i2sTx handle parameters */   
      hdma_SaiTx_2.Instance             = AUDIO_OUT2_SAI_DMAx_INSTANCE;  
      hdma_SaiTx_2.Init.Request             = AUDIO_OUT2_SAI_DMAx_REQUEST;  
      hdma_SaiTx_2.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_SaiTx_2.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_SaiTx_2.Init.MemInc              = DMA_MINC_ENABLE;
      hdma_SaiTx_2.Init.PeriphDataAlignment = AUDIO_OUT2_SAI_DMAx_PERIPH_DATA_SIZE;
      hdma_SaiTx_2.Init.MemDataAlignment    = AUDIO_OUT2_SAI_DMAx_MEM_DATA_SIZE;
      hdma_SaiTx_2.Init.Mode                = DMA_CIRCULAR;
      hdma_SaiTx_2.Init.Priority            = DMA_PRIORITY_HIGH;
      
      /* Associate the DMA handle */
      __HAL_LINKDMA(hSai, hdmatx, hdma_SaiTx_2);
      
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(&hdma_SaiTx_2);
      
      /* Configure the DMA Stream */
      HAL_DMA_Init(&hdma_SaiTx_2); 
      
      HAL_NVIC_DisableIRQ(AUDIO_OUT2_SAI_DMAx_IRQ);       
    }
    
    /*Reset pin configuration*/
    AUDIO_OUT2_RST_GPIO_CLK_ENABLE();
    
    /* Configure GPIO PINs to detect Interrupts */
    gpio_init_structure.Pin = AUDIO_OUT2_RST_PIN;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(AUDIO_OUT2_RST_GPIO_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN, GPIO_PIN_SET);     
    
    HAL_NVIC_SetPriority(AUDIO_OUT2_SAI_DMAx_IRQ, CCA01M1_AUDIO_OUT_IT_PRIORITY, 0);
  }
}

/**
* @brief  Deinitializes I2S MSP.
* @param  hi2s  I2S handle 
* @retval HAL status
*/
static void SAI_MspDeInit(SAI_HandleTypeDef *hSai)
{
  if(AudioOutCtx[0].Device == STA350BW_0)
  {       
    AUDIO_OUT1_SAI_CLK_DISABLE();  
    
    /* I2S3 pins de initialization: WS, SCK and SD pins -----------------------------*/

    HAL_GPIO_DeInit(AUDIO_OUT1_SAI_SCK_GPIO_PORT, AUDIO_OUT1_SAI_SCK_PIN);  
    HAL_GPIO_DeInit(AUDIO_OUT1_SAI_SD_GPIO_PORT, AUDIO_OUT1_SAI_SD_PIN);    
    HAL_GPIO_DeInit(AUDIO_OUT2_SAI_FS_GPIO_PORT, AUDIO_OUT1_SAI_FS_PIN);
    HAL_GPIO_DeInit(AUDIO_OUT1_SAI_MCK_GPIO_PORT, AUDIO_OUT1_SAI_MCK_PIN);   
    HAL_GPIO_DeInit(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN); 
    
    if(hSai->Instance == AUDIO_OUT1_SAI_INSTANCE)
    {     
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(hSai->hdmatx); 

      /* I2S DMA IRQ Channel configuration */
      HAL_NVIC_DisableIRQ(AUDIO_OUT1_SAI_DMAx_IRQ); 
    }    
  }
  else if (AudioOutCtx[0].Device == STA350BW_1)
  {   
    AUDIO_OUT1_SAI_CLK_DISABLE();  
    
    /* I2S3 pins de initialization: WS, SCK and SD pins -----------------------------*/

    HAL_GPIO_DeInit(AUDIO_OUT2_SAI_SCK_GPIO_PORT, AUDIO_OUT2_SAI_SCK_PIN);  
    HAL_GPIO_DeInit(AUDIO_OUT2_SAI_SD_GPIO_PORT, AUDIO_OUT2_SAI_SD_PIN);    
    HAL_GPIO_DeInit(AUDIO_OUT2_SAI_FS_GPIO_PORT, AUDIO_OUT2_SAI_FS_PIN);
    HAL_GPIO_DeInit(AUDIO_OUT2_SAI_MCK_GPIO_PORT, AUDIO_OUT2_SAI_MCK_PIN);   
    HAL_GPIO_DeInit(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN); 
    
    if(hSai->Instance == AUDIO_OUT2_SAI_INSTANCE)
    {     
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(hSai->hdmatx); 

      /* I2S DMA IRQ Channel configuration */
      HAL_NVIC_DisableIRQ(AUDIO_OUT2_SAI_DMAx_IRQ); 
    }    
  }
}


#else

/**
* @brief  hi2s clock Config.
* @param  hi2s hi2s handle
* @param  SampleRate  Audio frequency used to play the audio stream.
* @note   This API is called by CCA01M1_AUDIO_OUT_Init() and CCA01M1_AUDIO_OUT_SetFrequency()
*         Being __weak it can be overwritten by the application     
* @retval HAL status
*/
__weak HAL_StatusTypeDef MX_I2S_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t SampleRate)
{
  HAL_StatusTypeDef ret = HAL_OK;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);
  
#if (!defined(USE_STM32F0XX_NUCLEO) && !defined(USE_STM32L0XX_NUCLEO))
  
  RCC_PeriphCLKInitTypeDef rccclkinit;  
  /* Enable PLLI2S clock */
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
  
#if defined(STM32F411xE) || defined (STM32F446xx)
  rccclkinit.PLLI2S.PLLI2SM = 8;
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2;
  
#else
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
#endif
  
  if (SampleRate == 96000) 
  {
    rccclkinit.PLLI2S.PLLI2SN = 344;
    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else if (SampleRate == 48000) 
  {
    rccclkinit.PLLI2S.PLLI2SN = 258;
    rccclkinit.PLLI2S.PLLI2SR = 3;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else if (SampleRate == 44100)
  {
    rccclkinit.PLLI2S.PLLI2SN = 271;
    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else if (SampleRate == 32000)
  {
    rccclkinit.PLLI2S.PLLI2SN = 213;
    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else
  {
    ret = HAL_ERROR;
  } 
  
#endif 
  
  return ret;
}


/**
* @brief  Initializes the Audio Codec audio out instance (I2S).
* @param  MXConfig I2S configuration structure
* @note   Being __weak it can be overwritten by the application
* @retval HAL status
*/
__weak HAL_StatusTypeDef MX_I2S_OUT_Init(I2S_HandleTypeDef* hi2s, MX_I2S_OUT_Config *MXConfig)
{ 
  HAL_StatusTypeDef ret = HAL_OK;
  
  /* Disable I2S peripheral to allow access to I2S internal registers */
  __HAL_I2S_DISABLE(hi2s);
  
  /* Configure I2S */
  hi2s->Init.AudioFreq  	 = MXConfig->AudioFreq;  
  hi2s->Init.CPOL       	 = MXConfig->CPOL;
  hi2s->Init.DataFormat	 	 = MXConfig->DataFormat;
  hi2s->Init.MCLKOutput      = MXConfig->MCLKOutput;
  hi2s->Init.Mode            = MXConfig->Mode;
  hi2s->Init.Standard        = MXConfig->Standard;
  
#if (!defined(USE_STM32F0XX_NUCLEO) && !defined(USE_STM32L0XX_NUCLEO))
  hi2s->Init.ClockSource =  MXConfig->ClockSource;
#endif
  
  if(HAL_I2S_Init(hi2s) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  
  __HAL_I2S_ENABLE(hi2s);
  
  return ret;
}



/**
* @brief Tx Transfer completed callbacks
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_TransferComplete_CallBack(1);
  }
#ifndef USE_STM32L0XX_NUCLEO  
  if(hi2s->Instance == AUDIO_OUT2_I2S_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_TransferComplete_CallBack(1);
  }  
#endif
}

/**
* @brief Tx Transfer Half completed callbacks
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Manage the remaining file size and new address offset: This function 
  should be coded by user (its prototype is already declared in stm324xg_eval_audio.h) */  
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_HalfTransfer_CallBack(1);
  }
#ifndef USE_STM32L0XX_NUCLEO
  if(hi2s->Instance == AUDIO_OUT2_I2S_INSTANCE)
  {
    CCA01M1_AUDIO_OUT_HalfTransfer_CallBack(1);
  }
#endif
}

/**
* @brief  Get I2S MX Init configuration
* @param  hi2s I2S handle
* @param  MXConfig I2S configuration structure
* @retval None
*/
static void I2S_InitMXConfigStruct(I2S_HandleTypeDef* hi2s, MX_I2S_OUT_Config *MXConfig)
{
  MXConfig->AudioFreq  = hi2s->Init.AudioFreq; 
  MXConfig->CPOL       = hi2s->Init.CPOL;      
  MXConfig->DataFormat = hi2s->Init.DataFormat;
  MXConfig->MCLKOutput = hi2s->Init.MCLKOutput;
  MXConfig->Mode       = hi2s->Init.Mode;      
  MXConfig->Standard   = hi2s->Init.Standard ; 
  
#if (!defined(USE_STM32F0XX_NUCLEO) && !defined(USE_STM32L0XX_NUCLEO))  
  MXConfig->ClockSource = hi2s->Init.ClockSource;
#endif
  
}

/**
* @brief  Initialize CCA01M1_AUDIO_OUT MSP.
* @param  hi2s  I2S handle 
* @retval None
*/
static void I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
  GPIO_InitTypeDef  GPIO_InitStruct;  
  
  if(AudioOutCtx[1].Device == STA350BW_0)
  {
    static DMA_HandleTypeDef hdma_i2sTx_1;   
    
    AUDIO_OUT1_I2S_CLK_ENABLE();
    AUDIO_OUT1_I2S_SD_CLK_ENABLE(); 
    AUDIO_OUT1_I2S_SCK_CLK_ENABLE();
    AUDIO_OUT1_I2S_MCK_CLK_ENABLE();
    AUDIO_OUT1_I2S_WS_CLK_ENABLE();
    
    /* CODEC_I2S pins configuration: FS, SCK, MCK and SD pins ------------------*/
    GPIO_InitStruct.Pin = AUDIO_OUT1_I2S_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = AUDIO_OUT1_I2S_SCK_AF;
    HAL_GPIO_Init(AUDIO_OUT1_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
        
    GPIO_InitStruct.Alternate   = AUDIO_OUT1_I2S_SD_AF;
    GPIO_InitStruct.Pin         = AUDIO_OUT1_I2S_SD_PIN ;
    HAL_GPIO_Init(AUDIO_OUT1_I2S_SD_GPIO_PORT, &GPIO_InitStruct);	
        
    GPIO_InitStruct.Alternate   = AUDIO_OUT1_I2S_WS_AF;
    GPIO_InitStruct.Pin         = AUDIO_OUT1_I2S_WS_PIN ;
    HAL_GPIO_Init(AUDIO_OUT1_I2S_WS_GPIO_PORT, &GPIO_InitStruct); 
    
    GPIO_InitStruct.Alternate   = AUDIO_OUT1_I2S_MCK_AF;
    GPIO_InitStruct.Pin         = AUDIO_OUT1_I2S_MCK_PIN; 
    HAL_GPIO_Init(AUDIO_OUT1_I2S_MCK_GPIO_PORT, &GPIO_InitStruct);   
    
    /* Enable the I2S DMA clock */
    AUDIO_OUT1_I2S_DMAx_CLK_ENABLE(); 
    
    if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
    {
      /* Configure the hdma_i2sTx handle parameters */   
#ifdef USE_STM32L0XX_NUCLEO      
      hdma_i2sTx_1.Init.Request             = DMA_REQUEST_2;  
#endif
      
      hdma_i2sTx_1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_i2sTx_1.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_i2sTx_1.Init.MemInc              = DMA_MINC_ENABLE;
      hdma_i2sTx_1.Init.PeriphDataAlignment = AUDIO_OUT1_I2S_DMAx_PERIPH_DATA_SIZE;
      hdma_i2sTx_1.Init.MemDataAlignment    = AUDIO_OUT1_I2S_DMAx_MEM_DATA_SIZE;
      hdma_i2sTx_1.Init.Mode                = DMA_CIRCULAR;
      hdma_i2sTx_1.Init.Priority            = DMA_PRIORITY_HIGH;     
      hdma_i2sTx_1.Instance                 = AUDIO_OUT1_I2S_DMAx_STREAM;
      
#if (!defined(USE_STM32F0XX_NUCLEO) && !defined(USE_STM32L0XX_NUCLEO))
      hdma_i2sTx_1.Init.Channel             = AUDIO_OUT1_I2S_DMAx_CHANNEL;
      hdma_i2sTx_1.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
      hdma_i2sTx_1.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma_i2sTx_1.Init.MemBurst            = DMA_MBURST_SINGLE;
      hdma_i2sTx_1.Init.PeriphBurst         = DMA_PBURST_SINGLE;
#endif
      
      
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(&hdma_i2sTx_1);
      
      /* Associate the DMA handle */
      __HAL_LINKDMA(hi2s, hdmatx, hdma_i2sTx_1);
      
      /* Configure the DMA Stream */
      HAL_DMA_Init(&hdma_i2sTx_1);
      
      HAL_NVIC_DisableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ);
    }      
    
    /*Reset pin configuration*/
    AUDIO_OUT1_RST_GPIO_CLK_ENABLE();
    
    /* Configure GPIO PINs to detect Interrupts */
    GPIO_InitStruct.Pin = AUDIO_OUT1_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(AUDIO_OUT1_RST_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_SET);  
    
    HAL_NVIC_SetPriority(AUDIO_OUT1_I2S_DMAx_IRQ, CCA01M1_AUDIO_OUT_IT_PRIORITY, 0);
    
  }
#ifndef USE_STM32L0XX_NUCLEO  
  else if (AudioOutCtx[1].Device == STA350BW_1)
  {   
    static DMA_HandleTypeDef hdma_i2sTx_2;
    
    AUDIO_OUT2_I2S_CLK_ENABLE();
    AUDIO_OUT2_I2S_SD_CLK_ENABLE(); 
    AUDIO_OUT2_I2S_SCK_CLK_ENABLE();
    AUDIO_OUT2_I2S_MCK_CLK_ENABLE();
    AUDIO_OUT2_I2S_WS_CLK_ENABLE();
    
    /* I2S3 pins configuration: WS, SCK and SD pins -----------------------------*/
    GPIO_InitStruct.Pin         = AUDIO_OUT2_I2S_SCK_PIN ; 
    GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull        = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate   = AUDIO_OUT2_I2S_SCK_AF;
    HAL_GPIO_Init(AUDIO_OUT2_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Alternate   = AUDIO_OUT2_I2S_SD_AF;
    GPIO_InitStruct.Pin         = AUDIO_OUT2_I2S_SD_PIN;
    HAL_GPIO_Init(AUDIO_OUT2_I2S_SD_GPIO_PORT, &GPIO_InitStruct);     
    
    GPIO_InitStruct.Alternate   = AUDIO_OUT2_I2S_WS_AF;
    GPIO_InitStruct.Pin         = AUDIO_OUT2_I2S_WS_PIN;
    HAL_GPIO_Init(AUDIO_OUT2_I2S_WS_GPIO_PORT, &GPIO_InitStruct); 
    
    GPIO_InitStruct.Alternate   = AUDIO_OUT2_I2S_MCK_AF;
    GPIO_InitStruct.Pin         = AUDIO_OUT2_I2S_MCK_PIN; 
    HAL_GPIO_Init(AUDIO_OUT2_I2S_MCK_GPIO_PORT, &GPIO_InitStruct);  
    
    /* Enable the I2S DMA clock */
    AUDIO_OUT2_I2S_DMAx_CLK_ENABLE(); 
    
    if(hi2s->Instance == AUDIO_OUT2_I2S_INSTANCE)
    {
      /* Configure the hdma_i2sTx handle parameters */   
      hdma_i2sTx_2.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_i2sTx_2.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_i2sTx_2.Init.MemInc              = DMA_MINC_ENABLE;
      hdma_i2sTx_2.Init.PeriphDataAlignment = AUDIO_OUT2_I2S_DMAx_PERIPH_DATA_SIZE;
      hdma_i2sTx_2.Init.MemDataAlignment    = AUDIO_OUT2_I2S_DMAx_MEM_DATA_SIZE;
      hdma_i2sTx_2.Init.Mode                = DMA_CIRCULAR;
      hdma_i2sTx_2.Init.Priority            = DMA_PRIORITY_HIGH;     
      hdma_i2sTx_2.Instance                 = AUDIO_OUT2_I2S_DMAx_STREAM;
      
#if (!defined(USE_STM32F0XX_NUCLEO) && !defined(USE_STM32L0XX_NUCLEO))
      hdma_i2sTx_2.Init.Channel             = AUDIO_OUT2_I2S_DMAx_CHANNEL;
      hdma_i2sTx_2.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
      hdma_i2sTx_2.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma_i2sTx_2.Init.MemBurst            = DMA_MBURST_SINGLE;
      hdma_i2sTx_2.Init.PeriphBurst         = DMA_PBURST_SINGLE;
#endif
      
      /* Associate the DMA handle */
      __HAL_LINKDMA(hi2s, hdmatx, hdma_i2sTx_2);
      
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(&hdma_i2sTx_2);
      
      /* Configure the DMA Stream */
      HAL_DMA_Init(&hdma_i2sTx_2); 
      
      HAL_NVIC_DisableIRQ(AUDIO_OUT2_I2S_DMAx_IRQ);          
    }
    
    /*Reset pin configuration*/
    AUDIO_OUT2_RST_GPIO_CLK_ENABLE();
    
    /* Configure GPIO PINs to detect Interrupts */
    GPIO_InitStruct.Pin = AUDIO_OUT2_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(AUDIO_OUT2_RST_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN, GPIO_PIN_SET); 
    
    HAL_NVIC_SetPriority(AUDIO_OUT2_I2S_DMAx_IRQ, CCA01M1_AUDIO_OUT_IT_PRIORITY, 0);
  }
#endif
}

/**
* @brief  Deinitializes I2S MSP.
* @param  hi2s  I2S handle 
* @retval HAL status
*/
static void I2S_MspDeInit(I2S_HandleTypeDef *hi2s)
{
  if(AudioOutCtx[1].Device == STA350BW_0)
  {       
    AUDIO_OUT1_I2S_CLK_DISABLE();  
    
    /* I2S3 pins de initialization: WS, SCK and SD pins -----------------------------*/
    
    HAL_GPIO_DeInit(AUDIO_OUT1_I2S_SCK_GPIO_PORT, AUDIO_OUT1_I2S_SCK_PIN);  
    HAL_GPIO_DeInit(AUDIO_OUT1_I2S_SD_GPIO_PORT, AUDIO_OUT1_I2S_SD_PIN);    
    HAL_GPIO_DeInit(AUDIO_OUT1_I2S_WS_GPIO_PORT, AUDIO_OUT1_I2S_WS_PIN);
    HAL_GPIO_DeInit(AUDIO_OUT1_I2S_MCK_GPIO_PORT, AUDIO_OUT1_I2S_MCK_PIN);   
    HAL_GPIO_DeInit(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN); 
    
    if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
    {     
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(hi2s->hdmatx); 
      
      /* I2S DMA IRQ Channel configuration */
      HAL_NVIC_DisableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ); 
    }    
  }
#ifndef USE_STM32L0XX_NUCLEO  
  else if (AudioOutCtx[1].Device == STA350BW_1)
  {   
    AUDIO_OUT1_I2S_CLK_DISABLE();  
    
    /* I2S3 pins de initialization: WS, SCK and SD pins -----------------------------*/
    
    HAL_GPIO_DeInit(AUDIO_OUT2_I2S_SCK_GPIO_PORT, AUDIO_OUT2_I2S_SCK_PIN);  
    HAL_GPIO_DeInit(AUDIO_OUT2_I2S_SD_GPIO_PORT, AUDIO_OUT2_I2S_SD_PIN);    
    HAL_GPIO_DeInit(AUDIO_OUT2_I2S_WS_GPIO_PORT, AUDIO_OUT2_I2S_WS_PIN);
    HAL_GPIO_DeInit(AUDIO_OUT2_I2S_MCK_GPIO_PORT, AUDIO_OUT2_I2S_MCK_PIN);   
    HAL_GPIO_DeInit(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN); 
    
    if(hi2s->Instance == AUDIO_OUT2_I2S_INSTANCE)
    {     
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(hi2s->hdmatx); 
      
      /* I2S DMA IRQ Channel configuration */
      HAL_NVIC_DisableIRQ(AUDIO_OUT2_I2S_DMAx_IRQ); 
    }    
  }
#endif
}

#endif

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

