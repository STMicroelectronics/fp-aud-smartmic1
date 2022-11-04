/**
******************************************************************************
* @file    cca02m2_audio.c
* @author  SRA
* @version v1.1.2
* @date    10-Feb-2022
* @brief   This file provides the Audio driver for the cca02m2
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
#include "cca02m2_audio.h"
#include "cca02m2_conf.h"
#include "audio.h"

#ifndef USE_STM32L4XX_NUCLEO
#include "arm_math.h"
#endif

/** @addtogroup BSP
  * @{
  */

/** @addtogroup CCA02M2
  * @{
  */ 
  
/** @defgroup CCA02M2_AUDIO_ CCA02M2 AUDIO
  * @{
  */ 

/** @defgroup CCA02M2_AUDIO_Private_Macros CCA02M2_AUDIO_ Private Macros
  * @{
  */
/*### RECORD ###*/

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define DFSDM_OVER_SAMPLING(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (128U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (64U)  \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (32U) : (32U) 
	  
#define DFSDM_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (17U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (16U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (16U) : (16U)

#define DFSDM_FILTER_ORDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (DFSDM_FILTER_SINC5_ORDER) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (DFSDM_FILTER_SINC5_ORDER) : (DFSDM_FILTER_SINC5_ORDER)

#define DFSDM_MIC_BIT_SHIFT(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (5U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (8U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (10U) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (5U) : (5U)
 
#ifdef USE_STM32WBXX_NUCLEO
            
#define SAI_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (37U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (37U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (16U) \
        : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (16U) : (16U)            

#endif

/**
  * @}
  */ 

/** @defgroup CCA02M2_AUDIO_Exported_Variables CCA02M2_AUDIO_ Exported Variables
  * @{
  */
/* Recording context */
AUDIO_IN_Ctx_t                         AudioInCtx[AUDIO_IN_INSTANCES_NBR] = {0};

/**
  * @}
  */
  
/** @defgroup CCA02M2_AUDIO_Private_Variables CCA02M2_AUDIO_ Private Variables
  * @{
  */
#ifdef USE_STM32L4XX_NUCLEO

/* Recording handles */
static DFSDM_Channel_HandleTypeDef      hAudioInDfsdmChannel[4];
DMA_HandleTypeDef                       hDmaDfsdm[4]; 
static DFSDM_Filter_HandleTypeDef       hAudioInDfsdmFilter[4];

#else

#define DECIMATOR_NUM_TAPS (16U)
#define DECIMATOR_BLOCK_SIZE (16U*N_MS_PER_INTERRUPT)
#define DECIMATOR_FACTOR 2U
#define DECIMATOR_STATE_LENGTH (DECIMATOR_BLOCK_SIZE + (DECIMATOR_NUM_TAPS) -1U)
static arm_fir_decimate_instance_q15 ARM_Decimator_State[4];

/* PDM filters params */
static PDM_Filter_Handler_t  PDM_FilterHandler[4];
static PDM_Filter_Config_t   PDM_FilterConfig[4];

#ifdef USE_STM32WBXX_NUCLEO
SAI_HandleTypeDef            hAudioInSai;
#define PDM_INTERNAL_BUFFER_SIZE_SAI          ((MAX_MIC_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_TOTAL * N_MS_PER_INTERRUPT)
static uint16_t SAI_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_SAI];
#else
I2S_HandleTypeDef               hAudioInI2s;
static SPI_HandleTypeDef        hAudioInSPI;
static TIM_HandleTypeDef        TimDividerHandle;
static uint16_t I2S_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_I2S];
static uint16_t SPI_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_SPI];
            
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

#endif

#endif


/* Recording Buffer Trigger */
static __IO uint32_t    RecBuffTrigger          = 0;
static __IO uint32_t    RecBuffHalf             = 0;
static __IO uint32_t    MicBuffIndex[4];
#ifdef USE_STM32L4XX_NUCLEO
static int32_t          MicRecBuff[4][DEFAULT_AUDIO_IN_BUFFER_SIZE]; 
#endif

/**
  * @}
  */ 

/** @defgroup CCA02M2_AUDIO_Private_Function_Prototypes CCA02M2_AUDIO_ Private Function Prototypes
  * @{
  */
#ifdef USE_STM32L4XX_NUCLEO
/* DFSDM Channel Msp config */
static void DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hDfsdmChannel);
static void DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hDfsdmChannel);

/* DFSDM Filter Msp config */
static void DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter);
static void DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter);

/* DFSDM Filter conversion callbacks */
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
static void DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static void DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */

#else

#ifdef USE_STM32WBXX_NUCLEO
static void SAI_MspInit(SAI_HandleTypeDef *hsai);
#else
static HAL_StatusTypeDef AUDIO_IN_Timer_Init(void);
static HAL_StatusTypeDef AUDIO_IN_Timer_Start(void);
static void I2S_MspInit(I2S_HandleTypeDef *hi2s);
static void SPI_MspInit(SPI_HandleTypeDef *hspi);
#endif

#endif


/**
  * @}
  */ 

/** @defgroup CCA02M2_AUDIO_IN_Exported_Functions CCA02M2_AUDIO_IN Exported Functions
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
__weak int32_t CCA02M2_AUDIO_IN_Init(uint32_t Instance, CCA02M2_AUDIO_Init_t* AudioInit)
{  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;  
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
#ifdef USE_STM32L4XX_NUCLEO
      return  BSP_ERROR_WRONG_PARAM;
#else
      uint32_t PDM_Clock_Freq;     
      
      switch (AudioInit->SampleRate)
      {
      case AUDIO_FREQUENCY_8K:
        PDM_Clock_Freq = 1280;
        break;
        
      case AUDIO_FREQUENCY_16K:
        PDM_Clock_Freq = PDM_FREQ_16K;
        break;
        
      case AUDIO_FREQUENCY_32K:
        PDM_Clock_Freq = 2048;
        break;
        
      case AUDIO_FREQUENCY_48K:
        PDM_Clock_Freq = 3072;
        break;
        
      default:        
        PDM_Clock_Freq = 0;
        break;
      }
      
      if (PDM_Clock_Freq == 0U)
      {
        return BSP_ERROR_WRONG_PARAM;
      }
      
      AudioInCtx[Instance].DecimationFactor = (PDM_Clock_Freq * 1000U)/AudioInit->SampleRate;
      /* Double buffer for 1 microphone */
      AudioInCtx[Instance].Size = (PDM_Clock_Freq/8U) * 2U * N_MS_PER_INTERRUPT;
      
#ifdef USE_STM32WBXX_NUCLEO 
      
      if (AudioInCtx[Instance].ChannelsNbr == 1U)
      {
        AudioInCtx[Instance].Size *= 2U;
      }
      
      /* Initialize SAI */
      __HAL_SAI_RESET_HANDLE_STATE(&hAudioInSai);
      
      /* PLL clock is set depending by the AudioFreq */ 
      if(MX_SAI_ClockConfig(&hAudioInSai, PDM_Clock_Freq) != HAL_OK)
      {
        return  BSP_ERROR_CLOCK_FAILURE;
      }
      
      if(HAL_SAI_GetState(&hAudioInSai) == HAL_SAI_STATE_RESET)
      {
        SAI_MspInit(&hAudioInSai);
      }
      
      hAudioInSai.Instance = AUDIO_IN_SAI_INSTANCE;
      __HAL_SAI_DISABLE(&hAudioInSai);
      
      hAudioInSai.Init.Protocol = SAI_FREE_PROTOCOL;
      hAudioInSai.Init.AudioMode = SAI_MODEMASTER_RX;
      hAudioInSai.Init.DataSize = SAI_DATASIZE_16;   
      hAudioInSai.Init.FirstBit = SAI_FIRSTBIT_MSB;
      hAudioInSai.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
      hAudioInSai.Init.Synchro = SAI_ASYNCHRONOUS;
      hAudioInSai.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
      hAudioInSai.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
      hAudioInSai.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
      hAudioInSai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;    
      hAudioInSai.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
      hAudioInSai.Init.MonoStereoMode = SAI_STEREOMODE;
      hAudioInSai.Init.CompandingMode = SAI_NOCOMPANDING;
      hAudioInSai.Init.PdmInit.Activation = ENABLE;
      hAudioInSai.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK2_ENABLE;
      hAudioInSai.FrameInit.FrameLength = 16;
      if ( AudioInCtx[Instance].ChannelsNbr <= 2U)
      {        
        hAudioInSai.Init.PdmInit.MicPairsNbr = 1;
        hAudioInSai.Init.AudioFrequency = ((PDM_Clock_Freq * 1000U) / hAudioInSai.FrameInit.FrameLength ) * 2U;           
      }
      else
      {
        hAudioInSai.Init.PdmInit.MicPairsNbr = 2;
        hAudioInSai.Init.AudioFrequency = ((PDM_Clock_Freq * 1000U) / hAudioInSai.FrameInit.FrameLength ) * 4U;       
      }
      
      hAudioInSai.FrameInit.ActiveFrameLength = 1;
      hAudioInSai.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
      hAudioInSai.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
      hAudioInSai.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
      hAudioInSai.SlotInit.FirstBitOffset = 0;
      hAudioInSai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
      hAudioInSai.SlotInit.SlotNumber = 1; 
      hAudioInSai.SlotInit.SlotActive = 0x00000003;
      
      if (HAL_SAI_Init(&hAudioInSai) != HAL_OK)
      {
        return  BSP_ERROR_PERIPH_FAILURE;
      }      
      /* Enable SAI to generate clock used by audio driver */
      __HAL_SAI_ENABLE(&hAudioInSai);
      
#else             
      MX_I2S_IN_Config i2s_config;
      if(AudioInCtx[0].ChannelsNbr == 1U)
      {
        i2s_config.DataFormat = I2S_DATAFORMAT_16B;
      }
      else
      {
        i2s_config.DataFormat = I2S_DATAFORMAT_32B;
      }
      
      i2s_config.AudioFreq = ((PDM_Clock_Freq * 1000U) / 32U);
      i2s_config.CPOL = I2S_CPOL_HIGH;
      i2s_config.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
      i2s_config.Mode = I2S_MODE_MASTER_RX;
      i2s_config.Standard = I2S_STANDARD_MSB;
#ifdef USE_STM32F4XX_NUCLEO
      i2s_config.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
      i2s_config.ClockSource = I2S_CLOCK_PLL;
#else
      i2s_config.ClockSource = I2S_CLOCK_SYSCLK;
#endif
      
      if (AudioInCtx[0].ChannelsNbr>1U)
      {
        PDM_Clock_Freq *=2U;
        if (AUDIO_IN_Timer_Init() != HAL_OK)
        {
          return  BSP_ERROR_PERIPH_FAILURE;
        }
      }      
      /* PLL clock is set depending by the AudioFreq */ 
      if(MX_I2S_IN_ClockConfig(&hAudioInI2s, PDM_Clock_Freq) != HAL_OK)
      {
        return  BSP_ERROR_CLOCK_FAILURE;
      }      
      /* I2S Peripheral configuration */
      hAudioInI2s.Instance          = AUDIO_IN_I2S_INSTANCE;
      __HAL_I2S_DISABLE(&hAudioInI2s);
      I2S_MspInit(&hAudioInI2s);
      
      if (MX_I2S_IN_Init(&hAudioInI2s, &i2s_config)!= HAL_OK)
      {
        return  BSP_ERROR_PERIPH_FAILURE;
      }
      if (HAL_I2S_Init(&hAudioInI2s) != HAL_OK)
      {
        return  BSP_ERROR_PERIPH_FAILURE;
      }
      
      if (AudioInCtx[0].ChannelsNbr>2U)
      {
        /* Set the SPI parameters */
        hAudioInSPI.Instance = AUDIO_IN_SPI_INSTANCE;
        
        __HAL_SPI_DISABLE(&hAudioInSPI);
        SPI_MspInit(&hAudioInSPI);
        
        MX_SPI_Config spi_config;
        spi_config.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        spi_config.Direction = SPI_DIRECTION_2LINES_RXONLY;
        spi_config.CLKPhase = SPI_PHASE_2EDGE;
        spi_config.CLKPolarity = SPI_POLARITY_HIGH;
        spi_config.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        spi_config.CRCPolynomial = 7;
        spi_config.DataSize = SPI_DATASIZE_16BIT;
        spi_config.FirstBit = SPI_FIRSTBIT_MSB;
        spi_config.NSS = SPI_NSS_SOFT;
        spi_config.TIMode = SPI_TIMODE_DISABLED;
        spi_config.Mode = SPI_MODE_SLAVE;
        
        if (MX_SPI_Init(&hAudioInSPI, &spi_config)!= HAL_OK)
        {
          return  BSP_ERROR_PERIPH_FAILURE;
        }
        if (HAL_SPI_Init(&hAudioInSPI) != HAL_OK)
        {
          return  BSP_ERROR_PERIPH_FAILURE;
        }
      }
#endif
      if (CCA02M2_AUDIO_IN_PDMToPCM_Init(Instance, AudioInCtx[0].SampleRate, AudioInCtx[0].ChannelsNbr, AudioInCtx[0].ChannelsNbr)!= BSP_ERROR_NONE)
      {
        return  BSP_ERROR_NO_INIT;
      }
#endif
    }
    else if(Instance == 1U)
    {
#ifdef USE_STM32L4XX_NUCLEO
      
      int8_t i; 
      DFSDM_Filter_TypeDef* FilterInstnace[4] = {AUDIO_DFSDMx_MIC1_FILTER, AUDIO_DFSDMx_MIC2_FILTER, AUDIO_DFSDMx_MIC3_FILTER, AUDIO_DFSDMx_MIC4_FILTER};  
      DFSDM_Channel_TypeDef* ChannelInstance[4] = {AUDIO_DFSDMx_MIC1_CHANNEL, AUDIO_DFSDMx_MIC2_CHANNEL, AUDIO_DFSDMx_MIC3_CHANNEL, AUDIO_DFSDMx_MIC4_CHANNEL};
      uint32_t DigitalMicPins[4] = {DFSDM_CHANNEL_SAME_CHANNEL_PINS, DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS, DFSDM_CHANNEL_SAME_CHANNEL_PINS, DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS};
      uint32_t DigitalMicType[4] = {DFSDM_CHANNEL_SPI_RISING, DFSDM_CHANNEL_SPI_FALLING, DFSDM_CHANNEL_SPI_RISING, DFSDM_CHANNEL_SPI_FALLING};
      uint32_t Channel4Filter[4] = {AUDIO_DFSDMx_MIC1_CHANNEL_FOR_FILTER, AUDIO_DFSDMx_MIC2_CHANNEL_FOR_FILTER, AUDIO_DFSDMx_MIC3_CHANNEL_FOR_FILTER, AUDIO_DFSDMx_MIC4_CHANNEL_FOR_FILTER};
      MX_DFSDM_Config dfsdm_config;
      
      /* PLL clock is set depending on the AudioFreq (44.1khz vs 48khz groups) */
      if(MX_DFSDM1_ClockConfig(&hAudioInDfsdmChannel[0], AudioInit->SampleRate) != HAL_OK)
      {
        return  BSP_ERROR_CLOCK_FAILURE;
      }
      
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U)      
      /* Register the default DFSDM MSP callbacks */
      if(AudioInCtx[Instance].IsMspCallbacksValid == 0U)
      {
        if(CCA02M2_AUDIO_IN_RegisterDefaultMspCallbacks(Instance) != BSP_ERROR_NONE)
        {
          return  BSP_ERROR_PERIPH_FAILURE;
        }
      }
#else
      DFSDM_FilterMspInit(&hAudioInDfsdmFilter[1]);      
      DFSDM_ChannelMspInit(&hAudioInDfsdmChannel[1]);
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U) */
      
      for(i = 0; i < DFSDM_MIC_NUMBER; i ++)
      {
        dfsdm_config.FilterInstance  = FilterInstnace[i];
        dfsdm_config.ChannelInstance = ChannelInstance[i];
        dfsdm_config.DigitalMicPins  = DigitalMicPins[i];
        dfsdm_config.DigitalMicType  = DigitalMicType[i];
        dfsdm_config.Channel4Filter  = Channel4Filter[i];
        if((i == 0) && (AudioInCtx[Instance].Device == AUDIO_IN_DIGITAL_MIC))
        {
          dfsdm_config.RegularTrigger = DFSDM_FILTER_SW_TRIGGER;    
        } 
        else
        {
          dfsdm_config.RegularTrigger = DFSDM_FILTER_SYNC_TRIGGER;
        }
        dfsdm_config.SincOrder       = DFSDM_FILTER_ORDER(AudioInCtx[Instance].SampleRate);
        dfsdm_config.Oversampling    = DFSDM_OVER_SAMPLING(AudioInCtx[Instance].SampleRate);
        dfsdm_config.ClockDivider    = DFSDM_CLOCK_DIVIDER(AudioInCtx[Instance].SampleRate);
        dfsdm_config.RightBitShift   = DFSDM_MIC_BIT_SHIFT(AudioInCtx[Instance].SampleRate);
        
        if(((AudioInit->Device >> (uint8_t)i) & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
        {
          /* Default configuration of DFSDM filters and channels */
          if(MX_DFSDM1_Init(&hAudioInDfsdmFilter[i], &hAudioInDfsdmChannel[i], &dfsdm_config) != HAL_OK)
          {
            /* Return BSP_ERROR_PERIPH_FAILURE when operations are not correctly done */
            return  BSP_ERROR_PERIPH_FAILURE;
          }
          
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U)
          /* Register filter regular conversion callbacks */
          if(HAL_DFSDM_FILTER_RegisterCallback(&hAudioInDfsdmFilter[i], HAL_DFSDM_FILTER_REG_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
          {
            return  BSP_ERROR_PERIPH_FAILURE;
          }  
          if(HAL_DFSDM_FILTER_RegisterCallback(&hAudioInDfsdmFilter[i], HAL_DFSDM_FILTER_REG_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
          {
            return  BSP_ERROR_PERIPH_FAILURE;
          }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U) */
        }
      }
#else
      return  BSP_ERROR_WRONG_PARAM;
#endif
    }
    else /* Instance = 2 */
    {      
      // PDM direttamente?
    }
    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP; 
    /* Return BSP status */ 
  }
  return BSP_ERROR_NONE;
}

/**
* @brief  Deinit the audio IN peripherals.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/

__weak int32_t CCA02M2_AUDIO_IN_DeInit(uint32_t Instance)
{  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(Instance != 0U)
    {
#ifdef USE_STM32L4XX_NUCLEO
      
      int8_t i;
      for(i = 0; i < DFSDM_MIC_NUMBER; i++)
      {
        /* De-initializes DFSDM Filter handle */
        if(hAudioInDfsdmFilter[i].Instance != NULL)
        {
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0U)      
          DFSDM_FilterMspDeInit(&hAudioInDfsdmFilter[i]);          
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0U) */            
          if(HAL_OK != HAL_DFSDM_FilterDeInit(&hAudioInDfsdmFilter[i]))
          {
            return  BSP_ERROR_PERIPH_FAILURE;
          }
          hAudioInDfsdmFilter[i].Instance = NULL;
        }
        
        /* De-initializes DFSDM Channel handle */
        if(hAudioInDfsdmChannel[i].Instance != NULL)
        {
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0U)            
          DFSDM_ChannelMspDeInit(&hAudioInDfsdmChannel[i]);      
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0U) */                 
          if(HAL_OK != HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel[i]))
          {
            return  BSP_ERROR_PERIPH_FAILURE;
          }
          hAudioInDfsdmChannel[i].Instance = NULL;
        }
      }      
      /* Reset AudioInCtx[1].IsMultiBuff if any */
      AudioInCtx[1].IsMultiBuff = 0;      
#else
      return  BSP_ERROR_WRONG_PARAM;
#endif
    }
  /* Update BSP AUDIO IN state */     
  AudioInCtx[Instance].State = AUDIO_IN_STATE_RESET;   
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

#ifdef USE_STM32L4XX_NUCLEO

/**
* @brief  Clock Config.
* @param  hDfsdmChannel  DFSDM Channel Handle
* @param  SampleRate     Audio frequency to be configured for the DFSDM Channel.
* @note   This API is called by CCA02M2_AUDIO_IN_Init()
*         Being __weak it can be overwritten by the application     
* @retval HAL_status
*/
__weak HAL_StatusTypeDef MX_DFSDM1_ClockConfig(DFSDM_Channel_HandleTypeDef *hDfsdmChannel, uint32_t SampleRate)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmChannel);
  
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  
  switch (SampleRate) 
  {
  case AUDIO_FREQUENCY_8K: 
    {
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 37;
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 17;
      break;
    }
  case AUDIO_FREQUENCY_16K: 
    {
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
      break;
    }
  case AUDIO_FREQUENCY_32K: 
    {
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
      break;
    }
  case AUDIO_FREQUENCY_48K: 
    {
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
      break;
    }
  case AUDIO_FREQUENCY_96K: 
    {
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
      break;
    }
  default:
    {
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N = 43;
      RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P = 7;
      break;
    }
  }  
  /* Configure PLLSAI prescalers */
  /* Please note that some of these parameters must be consistent with 
  the parameters of the main PLL */
  RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  RCC_ExCLKInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;   
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1M = 6;  
  
  if(HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  return ret;
}


/**
* @brief  Initializes the Audio instance (DFSDM).
* @param  hDfsdmFilter  DFSDM Filter Handle
* @param  hDfsdmChannel DFSDM Channel Handle
* @param  SampleRate    Audio frequency to be configured for the DFSDM Channel.
* @note   Being __weak it can be overwritten by the application
* @note   Channel output Clock Divider and Filter Oversampling are calculated as follow: 
*         - Clock_Divider = CLK(input DFSDM)/CLK(micro) with
*           1MHZ < CLK(micro) < 3.2MHZ (TYP 2.4MHZ for MP34DT01TR)
*         - Oversampling = CLK(input DFSDM)/(Clock_Divider * AudioFreq)
* @retval HAL_status
*/
__weak HAL_StatusTypeDef MX_DFSDM1_Init(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel, MX_DFSDM_Config *MXConfig)
{  
  HAL_StatusTypeDef ret = HAL_OK;
  /* MIC filters  initialization */
  __HAL_DFSDM_FILTER_RESET_HANDLE_STATE(hDfsdmFilter); 
  hDfsdmFilter->Instance                          = MXConfig->FilterInstance; 
  hDfsdmFilter->Init.RegularParam.Trigger         = MXConfig->RegularTrigger;
  hDfsdmFilter->Init.RegularParam.FastMode        = ENABLE;
  hDfsdmFilter->Init.RegularParam.DmaMode         = ENABLE;
  hDfsdmFilter->Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
  hDfsdmFilter->Init.InjectedParam.ScanMode       = DISABLE;
  hDfsdmFilter->Init.InjectedParam.DmaMode        = DISABLE;
  hDfsdmFilter->Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
  hDfsdmFilter->Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
  hDfsdmFilter->Init.FilterParam.SincOrder        = MXConfig->SincOrder;
  hDfsdmFilter->Init.FilterParam.Oversampling     = MXConfig->Oversampling;   
  hDfsdmFilter->Init.FilterParam.IntOversampling  = 1;
  
  if(HAL_DFSDM_FilterInit(hDfsdmFilter) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }  
  
  /* MIC channels initialization */
  __HAL_DFSDM_CHANNEL_RESET_HANDLE_STATE(hDfsdmChannel);
  hDfsdmChannel->Instance                      = MXConfig->ChannelInstance;  
  hDfsdmChannel->Init.OutputClock.Activation   = ENABLE;
  hDfsdmChannel->Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO; 
  hDfsdmChannel->Init.OutputClock.Divider      = MXConfig->ClockDivider; 
  hDfsdmChannel->Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;  
  hDfsdmChannel->Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
  hDfsdmChannel->Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL; 
  hDfsdmChannel->Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
  hDfsdmChannel->Init.Awd.Oversampling         = 10; 
  hDfsdmChannel->Init.Offset                   = 0;
  hDfsdmChannel->Init.RightBitShift            = MXConfig->RightBitShift;
  hDfsdmChannel->Init.Input.Pins               = MXConfig->DigitalMicPins; 
  hDfsdmChannel->Init.SerialInterface.Type     = MXConfig->DigitalMicType;
  
  if(HAL_OK != HAL_DFSDM_ChannelInit(hDfsdmChannel))
  {
    ret =  HAL_ERROR;
  }  
  /* Configure injected channel */
  if(HAL_DFSDM_FilterConfigRegChannel(hDfsdmFilter, MXConfig->Channel4Filter, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    ret =  HAL_ERROR;
  } 
  
  return ret;
}


#if ((USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U) || (USE_HAL_SAI_REGISTER_CALLBACKS == 1U))
/**
* @brief Default BSP AUDIO IN Msp Callbacks
* @param Instance BSP AUDIO IN Instance
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_RegisterDefaultMspCallbacks (uint32_t Instance)
{
  uint32_t i;
  
  if(Instance == 1U)
  {    
    for(i = 0; i < DFSDM_MIC_NUMBER; i ++)
    {
      if(((AudioInCtx[Instance].Device >> i) & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
      {  
        __HAL_DFSDM_CHANNEL_RESET_HANDLE_STATE(&hAudioInDfsdmChannel[i]);
        __HAL_DFSDM_FILTER_RESET_HANDLE_STATE(&hAudioInDfsdmFilter[i]);
        
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)        
        /* Register MspInit/MspDeInit Callbacks */       
        if(HAL_DFSDM_CHANNEL_RegisterCallback(&hAudioInDfsdmChannel[i], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, DFSDM_ChannelMspInit) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
        else if(HAL_DFSDM_FILTER_RegisterCallback(&hAudioInDfsdmFilter[i], HAL_DFSDM_FILTER_MSPINIT_CB_ID, DFSDM_FilterMspInit) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
        else if(HAL_DFSDM_CHANNEL_RegisterCallback(&hAudioInDfsdmChannel[i], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, DFSDM_ChannelMspDeInit) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
        else if(HAL_DFSDM_FILTER_RegisterCallback(&hAudioInDfsdmFilter[i], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, DFSDM_FilterMspDeInit) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
        else
        {
        }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)  */         
      }
    }  
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  
  AudioInCtx[Instance].IsMspCallbacksValid = 1;
  /* Return BSP status */
  return BSP_ERROR_NONE;  
}

/**
* @brief BSP AUDIO In Filter Msp Callback registering
* @param Instance    AUDIO IN Instance
* @param CallBacks   pointer to filter MspInit/MspDeInit functions
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_RegisterMspCallbacks (uint32_t Instance, CCA02M2_AUDIO_IN_Cb_t *CallBacks)
{
  uint32_t i;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else 
  {  
    if(Instance == 1U)
    {
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)    
      for(i = 0U; i < DFSDM_MIC_NUMBER; i ++)
      { 
        __HAL_DFSDM_FILTER_RESET_HANDLE_STATE(&hAudioInDfsdmFilter[i]);
        
        /* Register MspInit/MspDeInit Callback */
        if(HAL_DFSDM_FILTER_RegisterCallback(&hAudioInDfsdmFilter[i], HAL_DFSDM_FILTER_MSPINIT_CB_ID, CallBacks->pMspFltrInitCb) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }  
        else if(HAL_DFSDM_FILTER_RegisterCallback(&hAudioInDfsdmFilter[i], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, CallBacks->pMspFltrDeInitCb) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }  
        else if(HAL_DFSDM_CHANNEL_RegisterCallback(&hAudioInDfsdmChannel[i], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, CallBacks->pMspChInitCb) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }  
        else if(HAL_DFSDM_CHANNEL_RegisterCallback(&hAudioInDfsdmChannel[i], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, CallBacks->pMspChDeInitCb) != HAL_OK)  
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
        else
        {
          return BSP_ERROR_WRONG_PARAM;
        }
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */  
    }
    else /* (Instance == 0U) */
    {
      return BSP_ERROR_WRONG_PARAM;
      
    }
    AudioInCtx[Instance].IsMspCallbacksValid = 1;
  }  
  /* Return BSP status */
  return BSP_ERROR_NONE; 
}
#endif /* ((USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) || (USE_HAL_SAI_REGISTER_CALLBACKS == 1U)) */

#else

#ifdef USE_STM32WBXX_NUCLEO


/**
* @brief  Clock Config.
* @param  hSai: SAI handle if required
* @param  SampleRate: Audio frequency used to play the audio stream.
* @note   This API is called by CCA02M2_AUDIO_IN_Init() 
*         Being __weak it can be overwritten by the application     
* @retval HAL_OK if no problem during execution, HAL_ERROR otherwise
*/
__weak HAL_StatusTypeDef MX_SAI_ClockConfig(SAI_HandleTypeDef *hSai, uint32_t PDM_rate)
{ 
  UNUSED(hSai);
  
  HAL_StatusTypeDef ret = HAL_OK;
  /*SAI PLL Configuration*/
  RCC_PeriphCLKInitTypeDef rccclkinit;
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit); 
  
  if ((PDM_rate % 1280U) == 0U)
  {
    rccclkinit.PLLSAI1.PLLN = 82;
    rccclkinit.PLLSAI1.PLLP = RCC_PLLP_DIV8;
  }
  else
  {
    rccclkinit.PLLSAI1.PLLN = 86;   
    rccclkinit.PLLSAI1.PLLP = RCC_PLLP_DIV7;  
  }    
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  
  if(HAL_RCCEx_PeriphCLKConfig(&rccclkinit) != HAL_OK)
  {
    ret = HAL_ERROR;
  }  
  return ret;
}

#else

/**
* @brief  Clock Config.
* @param  hi2s: I2S handle if required
* @param  SampleRate: Audio frequency used to play the audio stream.
* @note   This API is called by CCA02M2_AUDIO_IN_Init() 
*         Being __weak it can be overwritten by the application     
* @retval HAL_OK if no problem during execution, HAL_ERROR otherwise
*/
__weak HAL_StatusTypeDef MX_I2S_IN_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t PDM_rate)
{ 
  UNUSED(hi2s);
  
  HAL_StatusTypeDef ret = HAL_OK;
  /*I2S PLL Configuration*/
  RCC_PeriphCLKInitTypeDef rccclkinit;
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit); 
  
#if defined(STM32F446xx)
  rccclkinit.PLLI2S.PLLI2SQ = 2;
  rccclkinit.PLLI2SDivQ = 1;
#endif
  if ((PDM_rate % 1280U) == 0U)
  {
#if defined(STM32F411xE) || defined (STM32F446xx)    
    rccclkinit.PLLI2S.PLLI2SM = 10;
    rccclkinit.PLLI2S.PLLI2SN = 96;
#else
    rccclkinit.PLLI2S.PLLI2SN = 192;
#endif
    rccclkinit.PLLI2S.PLLI2SR = 5;
  }
  else
  {
#if defined(STM32F411xE) || defined (STM32F446xx)
    
    rccclkinit.PLLI2S.PLLI2SM = 8;
#endif
    rccclkinit.PLLI2S.PLLI2SN = 258;
    rccclkinit.PLLI2S.PLLI2SR = 3;
  }   
  
#if defined(STM32F446xx)
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2;
#else
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
#endif
  
  if(HAL_RCCEx_PeriphCLKConfig(&rccclkinit) != HAL_OK)
  {
    ret = HAL_ERROR;
  }  
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
  if (HAL_DMA_Init(&hdma_rx) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  
  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(hspi, hdmarx, hdma_rx);      
  
  return ret;
}


__weak HAL_StatusTypeDef MX_I2S_IN_Init(I2S_HandleTypeDef* hi2s, MX_I2S_IN_Config *MXConfig)
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
#ifdef USE_STM32F4XX_NUCLEO
  hi2s->Init.FullDuplexMode = MXConfig->FullDuplexMode;  
#endif
  
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
  HAL_NVIC_SetPriority(AUDIO_IN_I2S_DMAx_IRQ, CCA02M2_AUDIO_IN_IT_PRIORITY, CCA02M2_AUDIO_IN_IT_PRIORITY);
  HAL_NVIC_EnableIRQ(AUDIO_IN_I2S_DMAx_IRQ); 
  
  return ret;
}

#endif

#endif


#ifndef USE_STM32L4XX_NUCLEO

/**
* @brief  Initialize the PDM library.
* @param Instance    AUDIO IN Instance
* @param  AudioFreq  Audio sampling frequency
* @param  ChnlNbrIn  Number of input audio channels in the PDM buffer
* @param  ChnlNbrOut Number of desired output audio channels in the  resulting PCM buffer
* @retval BSP status
*/
__weak int32_t CCA02M2_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{  
  if(Instance != 0U)
  {
    return  BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
#ifdef USE_STM32L4XX_NUCLEO
    return  BSP_ERROR_WRONG_PARAM;
#else    
    uint32_t index; 
#if (ENABLE_HIGH_PERFORMANCE_MODE == 0U)    
    static int16_t aState_ARM[4][DECIMATOR_STATE_LENGTH];
    static int16_t aCoeffs[] = { -1406, 1634, -1943, 2386, -3080, 4325, -7223, 21690, 21690, -7223, 4325, -3080, 2386, -1943, 1634, -1406, };
#endif
    
    /* Enable CRC peripheral to unlock the PDM library */
    __HAL_RCC_CRC_CLK_ENABLE();
    
    for(index = 0; index < ChnlNbrIn; index++)
    {
      volatile uint32_t error = 0;
      /* Init PDM filters */
      PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
      if (ChnlNbrIn == 1U)
      {
        PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_BE; /* For WB this should be LE, TODO after bugfix in PDMlib */
      }
      else
      {
        PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
      }
      PDM_FilterHandler[index].high_pass_tap = 2122358088;
      PDM_FilterHandler[index].out_ptr_channels = (uint16_t)ChnlNbrOut;
      PDM_FilterHandler[index].in_ptr_channels  = (uint16_t)ChnlNbrIn;
      
      /* PDM lib config phase */
      PDM_FilterConfig[index].output_samples_number = (uint16_t) ((AudioFreq/1000U) * N_MS_PER_INTERRUPT);
      PDM_FilterConfig[index].mic_gain = 24;
      
      switch (AudioInCtx[0].DecimationFactor)
      {
#if (ENABLE_HIGH_PERFORMANCE_MODE == 1U)
      case 64:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64_HI_PERF;
        break;
      case 96:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_96_HI_PERF;
        break;
#else              
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
#endif
      default:
        break;
      }  

#if (ENABLE_HIGH_PERFORMANCE_MODE == 1U)
      switch(AudioInCtx[0].BitsPerSample)
      {
      case AUDIO_RESOLUTION_16b:
        PDM_FilterConfig[index].bit_depth = PDM_FILTER_BITDEPTH_16;
        break;
      case AUDIO_RESOLUTION_24b:
        PDM_FilterConfig[index].bit_depth = PDM_FILTER_BITDEPTH_24;
        break;
      case AUDIO_RESOLUTION_32b:
        PDM_FilterConfig[index].bit_depth = PDM_FILTER_BITDEPTH_24IN32;
        break;
      default:
        break;        
      }
#endif
      
      error = PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));
      if (error!=0U)
      {
        return  BSP_ERROR_NO_INIT;
      }      
      error = PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
      if (error!=0U)
      {
        return  BSP_ERROR_NO_INIT;
      }
    }    
#endif
  } 
  return BSP_ERROR_NONE;
}



/**
* @brief  Converts audio format from PDM to PCM.
* @param  Instance  AUDIO IN Instance  
* @param  PDMBuf    Pointer to PDM buffer data
* @param  PCMBuf    Pointer to PCM buffer data
* @retval BSP status
*/
__weak int32_t CCA02M2_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf)
{    
  if(Instance != 0U)
  {
    return  BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
#ifdef USE_STM32L4XX_NUCLEO    
    return  BSP_ERROR_WRONG_PARAM;
#else
    uint32_t index;
    
    for(index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
      if (AudioInCtx[Instance].SampleRate == 8000U)
      {
        uint16_t Decimate_Out[8U*N_MS_PER_INTERRUPT];
        uint32_t ii;
        uint16_t PDM_Filter_Out[16U*N_MS_PER_INTERRUPT];
        
        (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], PDM_Filter_Out, &PDM_FilterHandler[index]);
        (void)arm_fir_decimate_q15 (&ARM_Decimator_State[index], (q15_t *)&(PDM_Filter_Out), (q15_t*)&(Decimate_Out), DECIMATOR_BLOCK_SIZE);
        for (ii=0; ii<(8U*N_MS_PER_INTERRUPT); ii++)
        {
          PCMBuf[(ii * AudioInCtx[Instance].ChannelsNbr) + index] = Decimate_Out[ii];
        }
      }
      else
      {
        switch(AudioInCtx[Instance].BitsPerSample)
        {
        case AUDIO_RESOLUTION_16b:
          (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
          break;
        case AUDIO_RESOLUTION_24b:
          (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], &((uint8_t*)(PCMBuf))[3U*index], &PDM_FilterHandler[index]);          
          break;
        case AUDIO_RESOLUTION_32b:
          (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint32_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);          
          break;
        default:
          break;
        }
      }
    }    
#endif
  }  
  return BSP_ERROR_NONE;
}

#endif

/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  pbuf     Main buffer pointer for the recorded data storing  
* @param  NbrOfBytes     Size of the record buffer. Parameter not used when Instance is 0
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{  
  if(Instance >= (AUDIO_IN_INSTANCES_NBR - 1U) )
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    AudioInCtx[Instance].pBuff = (uint16_t*)pBuf;
    
    if(Instance == 0U)
    {
#ifdef USE_STM32L4XX_NUCLEO    
    return  BSP_ERROR_WRONG_PARAM;
#else
    
      UNUSED(NbrOfBytes);
      
#ifdef USE_STM32WBXX_NUCLEO
      
      if(HAL_SAI_Receive_DMA(&hAudioInSai, (uint8_t *)SAI_InternalBuffer, (uint16_t)(AudioInCtx[Instance].Size/2U * AudioInCtx[Instance].ChannelsNbr)) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }             
#else            
      if(AudioInCtx[Instance].ChannelsNbr > 2U)
      {
        if(HAL_SPI_Receive_DMA(&hAudioInSPI, (uint8_t *)SPI_InternalBuffer, (uint16_t)AudioInCtx[Instance].Size) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      if(AudioInCtx[Instance].ChannelsNbr != 1U)
      {
        if(AUDIO_IN_Timer_Start() != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      if(HAL_I2S_Receive_DMA(&hAudioInI2s, I2S_InternalBuffer, (uint16_t)AudioInCtx[Instance].Size/2U) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }         
#endif  
      /* Update BSP AUDIO IN state */     
      AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;           
#endif      
    }
    else
    {
#ifdef USE_STM32L4XX_NUCLEO      
      int32_t counter;  
      
      for (counter = (int32_t)(AudioInCtx[Instance].ChannelsNbr); counter > 0; counter --)
      {
        if(HAL_DFSDM_FilterRegularStart_DMA(&hAudioInDfsdmFilter[counter-1], MicRecBuff[counter-1], NbrOfBytes) != HAL_OK)  
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
      /* Update BSP AUDIO IN state */     
      AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;      
      
#else
      return BSP_ERROR_WRONG_PARAM;
#endif
    }
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_Stop(uint32_t Instance)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    if(Instance == 0U)
    {      
#ifdef USE_STM32L4XX_NUCLEO    
    return  BSP_ERROR_WRONG_PARAM;
#else
    
#ifdef USE_STM32WBXX_NUCLEO
       
      if(HAL_SAI_DMAStop(&hAudioInSai) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }      
#else      
      if(AudioInCtx[Instance].ChannelsNbr > 2U)
      {
        if(HAL_SPI_DMAStop(&hAudioInSPI)!= HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      if(HAL_I2S_DMAStop(&hAudioInI2s) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }      
#endif
#endif
    }
    else /*(Instance == 1U) */
    { 
#ifdef USE_STM32L4XX_NUCLEO
      int32_t counter;       
      for (counter = (int32_t)(AudioInCtx[Instance].ChannelsNbr); counter > 0; counter --)
      {
        if(HAL_DFSDM_FilterRegularStop_DMA(&hAudioInDfsdmFilter[counter-1]) != HAL_OK)  
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }      
#else
      return  BSP_ERROR_WRONG_PARAM;
#endif      
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
  } 
  /* Return BSP status */
  return BSP_ERROR_NONE;  
}


/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_Pause(uint32_t Instance)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else
  {  
    if(Instance == 0U)
    {            
#ifdef USE_STM32L4XX_NUCLEO    
    return  BSP_ERROR_WRONG_PARAM;
#else
    
#ifdef USE_STM32WBXX_NUCLEO      
            
      if(HAL_SAI_DMAPause(&hAudioInSai)!= HAL_OK)
      {
        return BSP_ERROR_WRONG_PARAM;
      }      
#else      
      if(HAL_I2S_DMAPause(&hAudioInI2s)!= HAL_OK)
      {
        return BSP_ERROR_WRONG_PARAM;
      }      
#endif
#endif
    }
    else /* (Instance == 1U) */
    {
#ifdef USE_STM32L4XX_NUCLEO
      int32_t counter;  
      
      for (counter = (int32_t)(AudioInCtx[Instance].ChannelsNbr); counter > 0; counter --)
      {
        if(HAL_DFSDM_FilterRegularStop_DMA(&hAudioInDfsdmFilter[counter-1]) != HAL_OK)  
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }     
#else
      return  BSP_ERROR_WRONG_PARAM;
#endif      
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_PAUSE;    
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

/**
* @brief  Resume the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_Resume(uint32_t Instance)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else 
  {
    if(Instance == 0U)
    { 
#ifdef USE_STM32L4XX_NUCLEO    
    return  BSP_ERROR_WRONG_PARAM;
#else
    
#ifdef USE_STM32WBXX_NUCLEO
      
      if(HAL_SAI_DMAResume(&hAudioInSai)!= HAL_OK)
      {
        return BSP_ERROR_WRONG_PARAM;
      }
#else
      if(HAL_I2S_DMAResume(&hAudioInI2s)!= HAL_OK)
      {
        return BSP_ERROR_WRONG_PARAM;
      }            
#endif
#endif
    }
    else /* (Instance == 1U) */
    {
#ifdef USE_STM32L4XX_NUCLEO
      int32_t counter;  
      
      for (counter = (int32_t)(AudioInCtx[Instance].ChannelsNbr); counter > 0; counter --)
      {
        if(HAL_DFSDM_FilterRegularStart_DMA(&hAudioInDfsdmFilter[counter-1], MicRecBuff[counter-1], DEFAULT_AUDIO_IN_BUFFER_SIZE) != HAL_OK)  
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }      
#else
      return  BSP_ERROR_WRONG_PARAM;
#endif
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

/**
* @brief  Starts audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  pBuf      Main buffer pointer for the recorded data storing
* @param  NbrOfBytes      Size of the recorded buffer. Parameter not used when Instance is 0
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes)
{
  if(Instance != 1U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
#ifdef USE_STM32L4XX_NUCLEO
    
    int8_t i;
    uint32_t mic_init[4] = {0};
    uint32_t audio_in_digital_mic = AUDIO_IN_DIGITAL_MIC1, pbuf_index = 0;
    uint32_t enabled_mic=0;
    
    /* Get the number of activated microphones */
    for(i = 0; i < DFSDM_MIC_NUMBER; i++)
    {
      if((AudioInCtx[Instance].Device & audio_in_digital_mic) == audio_in_digital_mic)
      {
        enabled_mic++;
      }
      audio_in_digital_mic = audio_in_digital_mic << 1;
    }
    
    AudioInCtx[Instance].pMultiBuff = pBuf;
    AudioInCtx[Instance].Size  = NbrOfBytes;
    AudioInCtx[Instance].IsMultiBuff = 1; 
    
    audio_in_digital_mic = AUDIO_IN_DIGITAL_MIC_LAST;
    for(i = 0; i < DFSDM_MIC_NUMBER; i++)
    {
      if( (mic_init[POS_VAL(audio_in_digital_mic)] != 1U) && ( (AudioInCtx[Instance].Device & audio_in_digital_mic) == audio_in_digital_mic) )
      {
        /* Call the Media layer start function for MICx channel */
        if(HAL_DFSDM_FilterRegularMsbStart_DMA(&hAudioInDfsdmFilter[POS_VAL(audio_in_digital_mic)], (int16_t*)pBuf[enabled_mic - 1U - pbuf_index], NbrOfBytes) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
        else
        {
          mic_init[POS_VAL(audio_in_digital_mic)] = 1;
          pbuf_index++;
        }
      }
      audio_in_digital_mic = audio_in_digital_mic >> 1;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
    /* Return BSP status */
    return BSP_ERROR_NONE; 
    
#else
    UNUSED(pBuf);
    UNUSED(NbrOfBytes);
    return BSP_ERROR_WRONG_PARAM;
#endif    
  }  
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital input device to be stopped
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device)
{
  /* Stop selected devices */
  int32_t ret = CCA02M2_AUDIO_IN_PauseChannels(Instance, Device);
  /* Update BSP AUDIO IN state */     
  AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;  
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be paused
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device)
{
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
#ifdef USE_STM32L4XX_NUCLEO
    
    uint32_t audio_in_digital_mic = AUDIO_IN_DIGITAL_MIC1; 
    int8_t i;  
    for(i = 0; i < DFSDM_MIC_NUMBER; i++)
    { 
      if((Device & audio_in_digital_mic) == audio_in_digital_mic)
      { 
        /* Call the Media layer stop function */
        if(HAL_DFSDM_FilterRegularStop_DMA(&hAudioInDfsdmFilter[POS_VAL(audio_in_digital_mic)]) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
      audio_in_digital_mic = audio_in_digital_mic << 1;      
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_PAUSE;   
    /* Return BSP status */
    return BSP_ERROR_NONE; 
#else
    return BSP_ERROR_WRONG_PARAM;
#endif    
  }        
}

/**
* @brief  Resume the audio file stream
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be resumed
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device)
{
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
#ifdef USE_STM32L4XX_NUCLEO
    
    uint32_t audio_in_digital_mic = AUDIO_IN_DIGITAL_MIC_LAST;  
    int8_t i;
    for(i = 0; i < DFSDM_MIC_NUMBER; i++)
    { 
      if((Device & audio_in_digital_mic) == audio_in_digital_mic)
      { 
        /* Start selected device channel */
        if(HAL_DFSDM_FilterRegularMsbStart_DMA(&hAudioInDfsdmFilter[POS_VAL(audio_in_digital_mic)],\
          (int16_t*)AudioInCtx[Instance].pMultiBuff[POS_VAL(audio_in_digital_mic)], AudioInCtx[Instance].Size) != HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
      audio_in_digital_mic = audio_in_digital_mic >> 1;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;  
    /* Return BSP status */
    return BSP_ERROR_NONE;  
#else
    return BSP_ERROR_WRONG_PARAM;
#endif    
  }  
}

/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN SAI PDM Instance. It can be only 2
* @param  pbuf     Main buffer pointer for the recorded data storing  
* @param  NbrOfBytes     Size of the record buffer. Parameter not used when Instance is 0
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{
  if(Instance != 2U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
#ifdef USE_STM32L4XX_NUCLEO
    
    static SAI_HandleTypeDef hAudioInSai;
    /* Start the process receive DMA */
    if(HAL_SAI_Receive_DMA(&hAudioInSai, (uint8_t*)pBuf, (uint16_t)(NbrOfBytes/(AudioInCtx[Instance].BitsPerSample/8U))) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING; 
    /* Return BSP status */
    return BSP_ERROR_NONE;
#else
    UNUSED(pBuf);
    UNUSED(NbrOfBytes);
    return BSP_ERROR_WRONG_PARAM;
#endif    
  }  
}


/**
* @brief  Set Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device to be used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
  CCA02M2_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {  
    if(Instance == 1U)
    {
#ifdef USE_STM32L4XX_NUCLEO
      
      int8_t i;
      for(i = 0; i < DFSDM_MIC_NUMBER; i ++)
      {      
        if(((Device >> (uint8_t)i) & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
        {
          if(HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel[i]) != HAL_OK)
          {
            return  BSP_ERROR_PERIPH_FAILURE;
          }
        }
      }      
#else
      return  BSP_ERROR_WRONG_PARAM;
#endif      
    }
    audio_init.Device = Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume;
    
    if(CCA02M2_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      return BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    return BSP_ERROR_BUSY;
  }  
  /* Return BSP status */  
  return BSP_ERROR_NONE;
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio Input Device */
    *Device = AudioInCtx[Instance].Device;
  }
  return BSP_ERROR_NONE;
}

/**
* @brief  Set Audio In frequency
* @param  Instance     Audio IN instance
* @param  SampleRate  Input frequency to be set
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t  SampleRate)
{
  CCA02M2_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
#ifdef USE_STM32L4XX_NUCLEO
      
      int8_t i;
      for(i = 0; i < DFSDM_MIC_NUMBER; i ++)
      {      
        if(((AudioInCtx[Instance].Device >> (uint8_t)i) & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
        {
          if(HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel[i]) != HAL_OK)
          {
            return  BSP_ERROR_PERIPH_FAILURE;
          }  
          if(HAL_DFSDM_FilterDeInit(&hAudioInDfsdmFilter[i]) != HAL_OK)
          {
            return  BSP_ERROR_PERIPH_FAILURE;
          }
        }
      }       
#else
      return  BSP_ERROR_WRONG_PARAM;
#endif      
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(CCA02M2_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      return BSP_ERROR_NO_INIT;
    }   
  }
  else
  {
    return BSP_ERROR_BUSY;
  }  
  /* Return BSP status */
  return BSP_ERROR_NONE;  
}

/**
* @brief  Get Audio In frequency
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  SampleRate  Audio Input frequency to be returned
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Return audio in frequency */
    *SampleRate = AudioInCtx[Instance].SampleRate;
  }  
  /* Return BSP status */  
  return BSP_ERROR_NONE;
}

/**
* @brief  Set Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be set
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  CCA02M2_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
#ifdef USE_STM32L4XX_NUCLEO
      
      int8_t i;
      for(i = 0; i < DFSDM_MIC_NUMBER; i ++)
      {      
        if(((AudioInCtx[Instance].Device >> (uint8_t)i) & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
        {
          if(HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel[i]) != HAL_OK)
          {
            return  BSP_ERROR_PERIPH_FAILURE;
          }
        }
      }      
#else
      return  BSP_ERROR_WRONG_PARAM;
#endif
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(CCA02M2_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      return BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    return BSP_ERROR_BUSY;
  }  
  /* Return BSP status */  
  return BSP_ERROR_NONE;
}

/**
* @brief  Get Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be returned
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio in resolution */
    *BitsPerSample = AudioInCtx[Instance].BitsPerSample;
  }
  return BSP_ERROR_NONE;
}

/**
* @brief  Set Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  if((Instance >= AUDIO_IN_INSTANCES_NBR) || (ChannelNbr > 2U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].ChannelsNbr = ChannelNbr;
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

/**
* @brief  Get Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Channel number to be returned */
    *ChannelNbr = AudioInCtx[Instance].ChannelsNbr;
  }
  return BSP_ERROR_NONE;
}

/**
* @brief  Set the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else if (Instance == 0U)
  {
#ifdef USE_STM32L4XX_NUCLEO
    return BSP_ERROR_WRONG_PARAM;
#else
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
#endif
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].Volume = Volume;
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;  
}

/**
* @brief  Get the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }  
  else
  {
    /* Input Volume to be returned */
    *Volume = AudioInCtx[Instance].Volume;
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;  
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  State     Audio Out state
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Input State to be returned */
    *State = AudioInCtx[Instance].State;
  }
  return BSP_ERROR_NONE;
}

#ifdef USE_STM32L4XX_NUCLEO

/**
  * @brief  CCA02M2 AUDIO IN interrupt handler.
  * @param  Instance Audio in instance.
  * @param  Device Device of the audio in stream.
  * @retval None.
  */
void CCA02M2_AUDIO_IN_IRQHandler(uint32_t Instance, uint32_t Device)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);

  if (Device == AUDIO_IN_DIGITAL_MIC1)
  {
    HAL_DMA_IRQHandler(hAudioInDfsdmFilter[0].hdmaReg);
  }
  else if (Device == AUDIO_IN_DIGITAL_MIC2)
  {
    HAL_DMA_IRQHandler(hAudioInDfsdmFilter[1].hdmaReg);
  }
  else if (Device == AUDIO_IN_DIGITAL_MIC3)
  {
    HAL_DMA_IRQHandler(hAudioInDfsdmFilter[2].hdmaReg);
  }
  else
  {
    HAL_DMA_IRQHandler(hAudioInDfsdmFilter[3].hdmaReg);
  }
}



#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0U)
/**
* @brief  Regular conversion complete callback. 
* @note   In interrupt mode, user has to read conversion value in this function
using HAL_DFSDM_FilterGetRegularValue.
* @param  hdfsdm_filter   DFSDM filter handle.
* @retval None
*/
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i, j;
  
  if(AudioInCtx[1].IsMultiBuff == 1U)
  {
    /* Call the record update function to get the second half */
    CCA02M2_AUDIO_IN_TransferComplete_CallBack(1);
  }
  else
  {   
    if(hdfsdm_filter == &hAudioInDfsdmFilter[POS_VAL(AUDIO_IN_DIGITAL_MIC1)])
    {
      for(j=0; j < AudioInCtx[1].ChannelsNbr; j ++)
      {
        for (i = 0; i < ((AudioInCtx[1].SampleRate / (uint32_t)1000) * N_MS_PER_INTERRUPT); i++)
        {
          AudioInCtx[1].HP_Filters[j].Z = ((MicRecBuff[j][i + ((AudioInCtx[1].SampleRate / (uint32_t)1000) * N_MS_PER_INTERRUPT)] /256) * (int32_t)(AudioInCtx[1].Volume)) /128;
          AudioInCtx[1].HP_Filters[j].oldOut = (0xFC * (AudioInCtx[1].HP_Filters[j].oldOut + AudioInCtx[1].HP_Filters[j].Z - AudioInCtx[1].HP_Filters[j].oldIn)) / 256;
          AudioInCtx[1].HP_Filters[j].oldIn = AudioInCtx[1].HP_Filters[j].Z;
          AudioInCtx[1].pBuff[(i * AudioInCtx[1].ChannelsNbr) + j] = (uint16_t) (SaturaLH(AudioInCtx[1].HP_Filters[j].oldOut, -32760, 32760));
        }		
      }	
      CCA02M2_AUDIO_IN_TransferComplete_CallBack(1);
    }
  }
}

/**
* @brief  Half regular conversion complete callback. 
* @param  hdfsdm_filter   DFSDM filter handle.
* @retval None
*/
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i, j;
  
  if(AudioInCtx[1].IsMultiBuff == 1U)
  {
    /* Call the record update function to get the first half */
    CCA02M2_AUDIO_IN_HalfTransfer_CallBack(1);
  }
  else
  {
    if(hdfsdm_filter == &hAudioInDfsdmFilter[POS_VAL(AUDIO_IN_DIGITAL_MIC1)])
    {
      for(j=0; j < AudioInCtx[1].ChannelsNbr; j ++)
      {
        for (i = 0; i < ((AudioInCtx[1].SampleRate / (uint32_t)1000)* N_MS_PER_INTERRUPT); i++)
        {
          AudioInCtx[1].HP_Filters[j].Z = ((MicRecBuff[j][i] /256) * (int32_t)(AudioInCtx[1].Volume)) /128;
          AudioInCtx[1].HP_Filters[j].oldOut = (0xFC * (AudioInCtx[1].HP_Filters[j].oldOut + AudioInCtx[1].HP_Filters[j].Z - AudioInCtx[1].HP_Filters[j].oldIn)) / 256;
          AudioInCtx[1].HP_Filters[j].oldIn = AudioInCtx[1].HP_Filters[j].Z;
          AudioInCtx[1].pBuff[(i * AudioInCtx[1].ChannelsNbr) + j] = (uint16_t) (SaturaLH(AudioInCtx[1].HP_Filters[j].oldOut, -32760, 32760));
        }		
      }	
      CCA02M2_AUDIO_IN_HalfTransfer_CallBack(1);
    }
  }
}

#endif

#else

#ifdef USE_STM32WBXX_NUCLEO

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the CCA02M2_AUDIO_IN_Start(...) function.
* @param hSai: SAI handle. Not used
* @retval None
*/
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hSai)
{
  UNUSED(hSai);
  uint32_t index;
  
  switch(AudioInCtx[0].ChannelsNbr)
  {
  case 1:
    {
      uint8_t * DataTempSAI = &(((uint8_t *)SAI_InternalBuffer)[AudioInCtx[0].Size/2U]) ;
      for(index = 0; index < (AudioInCtx[0].Size/4U) ; index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[2U*index]);
      }
      /* Remove after bugfix in PDMlib */
      for(index = 0; index < (AudioInCtx[0].Size/8U) ; index++)
      {
        ((uint16_t *)(AudioInCtx[0].pBuff))[index] = HTONS(((uint16_t *)(AudioInCtx[0].pBuff))[index]);
      }
      break;
    }    
  case 2:
    {  
      uint8_t * DataTempSAI = &(((uint8_t *)SAI_InternalBuffer)[AudioInCtx[0].Size]) ;
      for(index = 0; index < (AudioInCtx[0].Size) ; index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[index]);
      }
      break;
    }    
  case 4:
    {  
      uint8_t * DataTempSAI = &(((uint8_t *)SAI_InternalBuffer)[AudioInCtx[0].Size  * 2U]) ;
      for(index = 0; index < (AudioInCtx[0].Size * 2U) ; index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[index]);
      }
      break;
    }
  default:
    {
      break;
    }    
  }  
  CCA02M2_AUDIO_IN_TransferComplete_CallBack(0);
}

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the CCA02M2_AUDIO_IN_Start(...) function.
* @param hSai: SAI handle. Not used
* @retval None
*/
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hSai)
{
  UNUSED(hSai);
  uint32_t index;
  
  switch(AudioInCtx[0].ChannelsNbr)
  {
  case 1:
    {
      uint8_t * DataTempSAI = (uint8_t *)SAI_InternalBuffer;
      for(index = 0; index < (AudioInCtx[0].Size/4U) ; index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[2U*index]); 
      }
      /* Remove after bugfix in PDMlib */
      for(index = 0; index < (AudioInCtx[0].Size/8U) ; index++)
      {
        ((uint16_t *)(AudioInCtx[0].pBuff))[index] = HTONS(((uint16_t *)(AudioInCtx[0].pBuff))[index]);
      }
      break;
    }    
  case 2:
    {   
      uint8_t * DataTempSAI = (uint8_t *)SAI_InternalBuffer;
      for(index = 0; index < (AudioInCtx[0].Size); index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[index]); 
      }      
      break;
    }    
  case 4:
    {      
      uint8_t * DataTempSAI = (uint8_t *)SAI_InternalBuffer;
      for(index = 0; index < (AudioInCtx[0].Size * 2U); index++)
      {
        ((uint8_t *)(AudioInCtx[0].pBuff))[index] = (DataTempSAI[index]); 
      }
      break;
    }
  default:
    {      
      break;
    }    
  }
  CCA02M2_AUDIO_IN_HalfTransfer_CallBack(0);
}

#else

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the CCA02M2_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);
  uint32_t index;
  
  switch(AudioInCtx[0].ChannelsNbr)
  {
  case 1:
    {
      uint16_t * DataTempI2S = &I2S_InternalBuffer[AudioInCtx[0].Size/4U] ;
      for(index = 0; index < (AudioInCtx[0].Size/4U); index++)
      {
        AudioInCtx[0].pBuff[index] = (DataTempI2S[index]);
      }
      break;
    }
    
  case 2:
    {      
      uint16_t * DataTempI2S = &(I2S_InternalBuffer[AudioInCtx[0].Size/2U]);
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) 
      {
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
  CCA02M2_AUDIO_IN_TransferComplete_CallBack(0);
}

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the CCA02M2_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);
  uint32_t index;
  switch(AudioInCtx[0].ChannelsNbr)
  {
  case 1:
    {
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      for(index = 0; index < (AudioInCtx[0].Size/4U); index++)
      {
        AudioInCtx[0].pBuff[index] = (DataTempI2S[index]);
      }
      break;
    }    
  case 2:
    {      
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) 
      {
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
  CCA02M2_AUDIO_IN_HalfTransfer_CallBack(0);
}

#endif

#endif


/**
* @brief  User callback when record buffer is filled.
* @retval None
*/
__weak void CCA02M2_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
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
__weak void CCA02M2_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
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
__weak void CCA02M2_AUDIO_IN_Error_CallBack(uint32_t Instance)
{ 
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function is called when an Interrupt due to transfer error on or peripheral
  error occurs. */
}

/**
  * @}
  */ 
  
/** @defgroup CCA02M2_AUDIO_IN_Private_Functions CCA02M2_AUDIO_IN Private Functions
  * @{
  */ 

/*******************************************************************************
Static Functions
*******************************************************************************/
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U)
/**
* @brief  Regular conversion complete callback. 
* @note   In interrupt mode, user has to read conversion value in this function
using HAL_DFSDM_FilterGetRegularValue.
* @param  hdfsdm_filter   DFSDM filter handle.
* @retval None
*/
static void DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i, j = 0;
  uint32_t index;
  
  if(AudioInCtx[1].IsMultiBuff == 1U)
  {
    /* Call the record update function to get the second half */
    CCA02M2_AUDIO_IN_TransferComplete_CallBack(1);
  }
  else
  {   
    if(hdfsdm_filter == &hAudioInDfsdmFilter[POS_VAL(AUDIO_IN_DIGITAL_MIC1)])
    {      
      for(j=0; j < AudioInCtx[1].ChannelsNbr; j ++)
      {
        for (i = 0; i < (AudioInCtx[1].SampleRate / 1000); i++)
        {
          AudioInCtx[1].HP_Filters[j].Z = ((MicRecBuff[j][i + (AudioInCtx[1].SampleRate / 1000) ] >> 8) * AudioInCtx[1].Volume) >> 7;
          AudioInCtx[1].HP_Filters[j].oldOut = (0xFC * (AudioInCtx[1].HP_Filters[j].oldOut + AudioInCtx[1].HP_Filters[j].Z - AudioInCtx[1].HP_Filters[j].oldIn)) / 256;
          AudioInCtx[1].HP_Filters[j].oldIn = AudioInCtx[1].HP_Filters[j].Z;
          AudioInCtx[1].pBuff[i * AudioInCtx[1].ChannelsNbr + j] = SaturaLH(AudioInCtx[1].HP_Filters[j].oldOut, -32760, 32760);
          RecBuffTrigger +=1U;
        }		
      }	
    }    
    /* Call Half Transfer Complete callback */
    if(RecBuffTrigger == (AudioInCtx[1].Size/2U))
    {
      if(RecBuffHalf == 0U)
      {
        RecBuffHalf = 1;  
        CCA02M2_AUDIO_IN_HalfTransfer_CallBack(1);
      }
    }
    /* Call Transfer Complete callback */
    if(RecBuffTrigger == AudioInCtx[1].Size)
    {
      /* Reset Application Buffer Trigger */
      RecBuffTrigger = 0;
      RecBuffHalf = 0; 
      /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
      CCA02M2_AUDIO_IN_TransferComplete_CallBack(1);
    }
  }
}

/**
* @brief  Half regular conversion complete callback. 
* @param  hdfsdm_filter   DFSDM filter handle.
* @retval None
*/
static void DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i, j = 0;
  uint32_t index;
  
  if(AudioInCtx[1].IsMultiBuff == 1U)
  {
    /* Call the record update function to get the first half */
    CCA02M2_AUDIO_IN_HalfTransfer_CallBack(1);
  }
  else
    if(hdfsdm_filter == &hAudioInDfsdmFilter[POS_VAL(AUDIO_IN_DIGITAL_MIC1)])
    {      
      for(j=0; j < AudioInCtx[1].ChannelsNbr; j ++)
      {
        for (i = 0; i < (AudioInCtx[1].SampleRate / 1000); i++)
        {
          AudioInCtx[1].HP_Filters[j].Z = ((MicRecBuff[j][i] >> 8) * AudioInCtx[1].Volume) >> 7;
          AudioInCtx[1].HP_Filters[j].oldOut = (0xFC * (AudioInCtx[1].HP_Filters[j].oldOut + AudioInCtx[1].HP_Filters[j].Z - AudioInCtx[1].HP_Filters[j].oldIn)) / 256;
          AudioInCtx[1].HP_Filters[j].oldIn = AudioInCtx[1].HP_Filters[j].Z;
          AudioInCtx[1].pBuff[i * AudioInCtx[1].ChannelsNbr + j] = SaturaLH(AudioInCtx[1].HP_Filters[j].oldOut, -32760, 32760);
          RecBuffTrigger +=1U;
        }		
      }	
    }  
  /* Call Half Transfer Complete callback */
  if(RecBuffTrigger == (AudioInCtx[1].Size/2U))
  {
    if(RecBuffHalf == 0U)
    {
      RecBuffHalf = 1;  
      CCA02M2_AUDIO_IN_HalfTransfer_CallBack(1);
    }
  }
  /* Call Transfer Complete callback */
  if(RecBuffTrigger == AudioInCtx[1].Size)
  {
    /* Reset Application Buffer Trigger */
    RecBuffTrigger = 0;
    RecBuffHalf = 0; 
    /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
    CCA02M2_AUDIO_IN_TransferComplete_CallBack(1);
  }
}

#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U) */

#ifdef USE_STM32L4XX_NUCLEO

/**
* @brief  Initialize the DFSDM channel MSP.
* @param  hDfsdmChannel DFSDM Channel handle
* @retval None
*/
static void DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hDfsdmChannel)
{
  GPIO_InitTypeDef  GPIO_InitStruct;  
  
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmChannel);
  
  /* Enable DFSDM clock */                                  
  AUDIO_DFSDMx_CLK_ENABLE();                
  /* Enable GPIO clock */ 
  AUDIO_DFSDMx_CKOUT_GPIO_CLK_ENABLE(); 
  AUDIO_DFSDMx_DATIN_MIC1_GPIO_CLK_ENABLE();
  AUDIO_DFSDMx_DATIN_MIC2_GPIO_CLK_ENABLE();
  
  /* DFSDM pins configuration: DFSDM_CKOUT, DMIC_DATIN pins ------------------*/
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_CKOUT_PIN;
  GPIO_InitStruct.Alternate = AUDIO_DFSDMx_CKOUT_AF;
  HAL_GPIO_Init(AUDIO_DFSDMx_CKOUT_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC1_PIN;
  GPIO_InitStruct.Alternate = AUDIO_DFSDMx_DATIN_MIC1_AF;
  HAL_GPIO_Init(AUDIO_DFSDMx_DATIN_MIC1_GPIO_PORT, &GPIO_InitStruct); 
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC2_PIN;
  GPIO_InitStruct.Alternate = AUDIO_DFSDMx_DATIN_MIC2_AF;
  HAL_GPIO_Init(AUDIO_DFSDMx_DATIN_MIC2_GPIO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC3_PIN;
  GPIO_InitStruct.Alternate = AUDIO_DFSDMx_DATIN_MIC3_AF;
  HAL_GPIO_Init(AUDIO_DFSDMx_DATIN_MIC3_GPIO_PORT, &GPIO_InitStruct); 
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC4_PIN;
  GPIO_InitStruct.Alternate = AUDIO_DFSDMx_DATIN_MIC4_AF;
  HAL_GPIO_Init(AUDIO_DFSDMx_DATIN_MIC4_GPIO_PORT, &GPIO_InitStruct);
}

/**
* @brief  DeInitialize the DFSDM channel MSP.
* @param  hDfsdmChannel DFSDM Channel handle
* @retval None
*/
static void DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hDfsdmChannel)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmChannel);
  
  GPIO_InitTypeDef  GPIO_InitStruct;  
  /* DFSDM pins configuration: DFSDM_CKOUT, DMIC_DATIN pins ------------------*/
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_CKOUT_PIN;
  HAL_GPIO_DeInit(AUDIO_DFSDMx_CKOUT_GPIO_PORT, GPIO_InitStruct.Pin);    
  
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC1_PIN;
  HAL_GPIO_DeInit(AUDIO_DFSDMx_DATIN_MIC1_GPIO_PORT, GPIO_InitStruct.Pin);
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC2_PIN;
  HAL_GPIO_DeInit(AUDIO_DFSDMx_DATIN_MIC2_GPIO_PORT, GPIO_InitStruct.Pin);
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC3_PIN;
  HAL_GPIO_DeInit(AUDIO_DFSDMx_DATIN_MIC3_GPIO_PORT, GPIO_InitStruct.Pin);
  GPIO_InitStruct.Pin = AUDIO_DFSDMx_DATIN_MIC4_PIN;
  HAL_GPIO_DeInit(AUDIO_DFSDMx_DATIN_MIC4_GPIO_PORT, GPIO_InitStruct.Pin);
}

/**
* @brief  Initialize the DFSDM filter MSP.
* @param  hDfsdmFilter DFSDM Filter handle
* @retval None
*/
static void DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter)
{   
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmFilter); 
  
  uint32_t mic_init[4] = {0};
  uint32_t mic_num; 
  int8_t i;
  DMA_Channel_TypeDef* AUDIO_DFSDMx_DMAx_MIC_STREAM[4] = {AUDIO_DFSDMx_DMAx_MIC1_STREAM, AUDIO_DFSDMx_DMAx_MIC2_STREAM, AUDIO_DFSDMx_DMAx_MIC3_STREAM, AUDIO_DFSDMx_DMAx_MIC4_STREAM};
  
  /* Enable DFSDM clock */
  AUDIO_DFSDMx_CLK_ENABLE();                
  /* Enable the DMA clock */ 
  AUDIO_DFSDMx_DMAx_CLK_ENABLE();
  
  for(i = 0; i < DFSDM_MIC_NUMBER; i++)
  {
    if((mic_init[POS_VAL(AUDIO_IN_DIGITAL_MIC1)] != 1U) && ((AudioInCtx[1].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) )
    {
      mic_num = POS_VAL(AUDIO_IN_DIGITAL_MIC1);
      mic_init[mic_num] = 1;
    }
    else if((mic_init[POS_VAL(AUDIO_IN_DIGITAL_MIC2)] != 1U) && ((AudioInCtx[1].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) )
    {
      mic_num = POS_VAL(AUDIO_IN_DIGITAL_MIC2);
      mic_init[mic_num] = 1;
    }
    else if((mic_init[POS_VAL(AUDIO_IN_DIGITAL_MIC3)] != 1U) && ((AudioInCtx[1].Device & AUDIO_IN_DIGITAL_MIC3) == AUDIO_IN_DIGITAL_MIC3) )
    {
      mic_num = POS_VAL(AUDIO_IN_DIGITAL_MIC3);
      mic_init[mic_num] = 1;
    }
    else if((mic_init[POS_VAL(AUDIO_IN_DIGITAL_MIC4)] != 1U) && ((AudioInCtx[1].Device & AUDIO_IN_DIGITAL_MIC4) == AUDIO_IN_DIGITAL_MIC4) )
    {
      mic_num = POS_VAL(AUDIO_IN_DIGITAL_MIC4);
      mic_init[mic_num] = 1;
    }
    else
    {
      break;
    }  
    /* Configure the hDmaDfsdm[i] handle parameters */
    hDmaDfsdm[mic_num].Init.Request             = DMA_REQUEST_0; 
    hDmaDfsdm[mic_num].Instance                 = AUDIO_DFSDMx_DMAx_MIC_STREAM[mic_num];
    hDmaDfsdm[mic_num].Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDmaDfsdm[mic_num].Init.PeriphInc           = DMA_PINC_DISABLE;
    hDmaDfsdm[mic_num].Init.MemInc              = DMA_MINC_ENABLE;
    hDmaDfsdm[mic_num].Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hDmaDfsdm[mic_num].Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hDmaDfsdm[mic_num].Init.Mode                = DMA_CIRCULAR;
    hDmaDfsdm[mic_num].Init.Priority            = DMA_PRIORITY_HIGH;  
    hDmaDfsdm[mic_num].State                    = HAL_DMA_STATE_RESET;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(&hAudioInDfsdmFilter[mic_num], hdmaReg, hDmaDfsdm[mic_num]);
    
    /* Reset DMA handle state */
    __HAL_DMA_RESET_HANDLE_STATE(&hDmaDfsdm[mic_num]);
    
    /* Configure the DMA Channel */
    (void)HAL_DMA_Init(&hDmaDfsdm[mic_num]);
    
    if (hDmaDfsdm[mic_num].Instance == AUDIO_DFSDMx_DMAx_MIC1_STREAM)
    {
      /* DMA IRQ Channel configuration */
      HAL_NVIC_SetPriority(AUDIO_DFSDMx_DMAx_MIC1_IRQ, CCA02M2_AUDIO_IN_IT_PRIORITY, CCA02M2_AUDIO_IN_IT_PRIORITY);
      HAL_NVIC_EnableIRQ(AUDIO_DFSDMx_DMAx_MIC1_IRQ);
    }
  }
}

/**
* @brief  DeInitialize the DFSDM filter MSP.
* @param  hDfsdmFilter DFSDM Filter handle
* @retval None
*/
static void DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmFilter);
  
  int8_t i;
  /* Configure the DMA Channel */
  for(i = 0; i < DFSDM_MIC_NUMBER; i++)
  {
    if(hDmaDfsdm[i].Instance != NULL)
    {
      (void)HAL_DMA_DeInit(&hDmaDfsdm[i]); 
    }
  } 
}

#else

#ifdef USE_STM32WBXX_NUCLEO
/**
* @brief AUDIO IN SAI MSP Init
* @param None
* @retval None
*/
 void SAI_MspInit(SAI_HandleTypeDef *hsai)
 {
   static DMA_HandleTypeDef hSaiDma;
   
   GPIO_InitTypeDef  GPIO_InitStruct;
   
   /* Enable the SAI peripheral clock */
   AUDIO_IN_SAI_CLK_ENABLE();  
   /* Enable SAI GPIO clocks */
   AUDIO_IN_SAI_SCK_GPIO_CLK_ENABLE();
   AUDIO_IN_SAI_SD_GPIO_CLK_ENABLE();  
   
   /* SAI pins configuration: SCK and SD pins ------------------------------*/
   GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull      = GPIO_PULLUP;
   GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
   
   GPIO_InitStruct.Pin       = AUDIO_IN_SAI_SCK_PIN;
   GPIO_InitStruct.Alternate = AUDIO_IN_SAI_SCK_AF;
   HAL_GPIO_Init(AUDIO_IN_SAI_SCK_GPIO_PORT, &GPIO_InitStruct);
   
   GPIO_InitStruct.Pin       = AUDIO_IN_SAI_SD_PIN ;
   GPIO_InitStruct.Alternate = AUDIO_IN_SAI_SD_AF;
   HAL_GPIO_Init(AUDIO_IN_SAI_SD_GPIO_PORT, &GPIO_InitStruct);
   
   GPIO_InitStruct.Pin     = AUDIO_IN_SAI_SD2_PIN ;
   GPIO_InitStruct.Alternate = AUDIO_IN_SAI_SD2_AF;
   HAL_GPIO_Init(AUDIO_IN_SAI_SD_GPIO_PORT, &GPIO_InitStruct);
   
   /* Enable the DMA clock */
   /* DMA controller clock enable */
   __HAL_RCC_DMAMUX1_CLK_ENABLE();
   __HAL_RCC_DMA1_CLK_ENABLE();  
   
   /* Configure the hSaiDma handle parameters */
   hSaiDma.Instance = DMA1_Channel1; /*tODO DEFINES */
   hSaiDma.Init.Request             = DMA_REQUEST_SAI1_A;
   hSaiDma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
   hSaiDma.Init.PeriphInc           = DMA_PINC_DISABLE;
   hSaiDma.Init.MemInc              = DMA_MINC_ENABLE;
   hSaiDma.Init.PeriphDataAlignment = AUDIO_IN_SAI_DMAx_PERIPH_DATA_SIZE;
   hSaiDma.Init.MemDataAlignment    = AUDIO_IN_SAI_DMAx_MEM_DATA_SIZE;
   hSaiDma.Init.Mode                = DMA_CIRCULAR;
   hSaiDma.Init.Priority            = DMA_PRIORITY_HIGH;
   
   /* Associate the DMA handle */
   __HAL_LINKDMA(hsai, hdmarx, hSaiDma);    
   /* Deinitialize the Stream for new transfer */
   (void)HAL_DMA_DeInit(&hSaiDma);    
   /* Configure the DMA Stream */
   (void)HAL_DMA_Init(&hSaiDma);
   
   /* I2S DMA IRQ Channel configuration */
   HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, CCA02M2_AUDIO_IN_IT_PRIORITY, CCA02M2_AUDIO_IN_IT_PRIORITY);
   HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
 }

#else
static void I2S_MspInit(I2S_HandleTypeDef *hi2s)
{	
  UNUSED(hi2s);
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the I2S2 peripheral clock */
  AUDIO_IN_I2S_CLK_ENABLE();  
  /* Enable I2S GPIO clocks */
  AUDIO_IN_I2S_SCK_GPIO_CLK_ENABLE();
  AUDIO_IN_I2S_MOSI_GPIO_CLK_ENABLE();
  
  /* I2S2 pins configuration: SCK and MOSI pins ------------------------------*/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  
  GPIO_InitStruct.Pin       = AUDIO_IN_I2S_SCK_PIN;
  GPIO_InitStruct.Alternate = AUDIO_IN_I2S_SCK_AF;
  HAL_GPIO_Init(AUDIO_IN_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
  
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

#endif

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

/**
* @}
*/ 
