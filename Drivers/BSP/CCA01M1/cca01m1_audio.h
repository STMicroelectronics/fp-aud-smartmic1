/**
  ******************************************************************************
  * @file    cca01m1_audio.h
  * @author  SRA
  * @brief   This file contains definitions for cca01m1_audio.c
  *          firmware driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CCA01M1_AUDIO_H
#define CCA01M1_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
   
#include <stdio.h>
#include <string.h>
#include "cca01m1_conf.h"
#include "audio.h"  
#include "sta350bw.h" 
  
/** @addtogroup BSP
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1 X_NUCLEO_CCA01M1
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1_AUDIO X_NUCLEO_CCA01M1_AUDIO
* @{
*/  

/** @addtogroup X_NUCLEO_CCA01M1_AUDIO_Exported_Types Exported types
* @{
*/
typedef enum
{
  CODEC_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  STA350BW_0 = 0,                   
  STA350BW_1 = 1
} CODEC_ID_t;

typedef struct
{                                   
  uint32_t                    Device;                                           
  uint32_t                    SampleRate;                                         
  uint32_t                    BitsPerSample;                                          
  uint32_t                    ChannelsNbr;                                         
  uint32_t                    Volume;
}CCA01M1_AUDIO_Init_t;

typedef struct
{
  uint32_t                    Instance;            /* Audio OUT instance              */  
  int32_t                     Device;              /* Audio OUT device to be used     */ 
  uint32_t                    SampleRate;          /* Audio OUT Sample rate           */
  uint32_t                    BitsPerSample;       /* Audio OUT Sample Bit Per Sample */
  uint32_t                    Volume;              /* Audio OUT volume                */
  uint32_t                    ChannelsNbr;         /* Audio OUT number of channel     */
  uint32_t                    IsMute;              /* Mute state                      */   
  uint32_t                    State;               /* Audio OUT State                 */
  uint32_t                    IsMspCallbacksValid; /* Is Msp Callbacks registred      */ 
}AUDIO_OUT_Ctx_t;

#ifdef USE_STM32L4XX_NUCLEO

typedef struct
{
  uint32_t AudioFrequency;
  uint32_t AudioMode;
  uint32_t DataSize;
  uint32_t Mckdiv;
  uint32_t MonoStereoMode;
  uint32_t ClockStrobing;
  uint32_t Synchro;
  uint32_t OutputDrive;
  uint32_t SynchroExt;
  uint32_t FrameLength;
  uint32_t ActiveFrameLength;
  uint32_t SlotActive; 
}MX_SAI_Config;

extern SAI_HandleTypeDef hAudioOut[];

#else

typedef struct
{
  uint32_t AudioFreq;
  uint32_t ClockSource;
  uint32_t CPOL;      
  uint32_t DataFormat;
  uint32_t MCLKOutput;
  uint32_t Mode;      
  uint32_t Standard;  
}MX_I2S_OUT_Config;

extern I2S_HandleTypeDef hAudioOut[];

#endif

/**
* @}
*/ 

/** @defgroup  X_NUCLEO_CCA01M1_AUDIO_Public_Constants Public Constants 
* @{
*/
#ifdef USE_STM32L0XX_NUCLEO
#define SOUNDTERMINAL_DEVICE_NBR   1
#else
#define SOUNDTERMINAL_DEVICE_NBR   2
#endif
  
  /* AUDIO FREQUENCY */
#ifndef AUDIO_FREQUENCY_192K
#define AUDIO_FREQUENCY_192K     (uint32_t)192000U
#endif
#ifndef AUDIO_FREQUENCY_176K  
#define AUDIO_FREQUENCY_176K     (uint32_t)176400U
#endif
#ifndef AUDIO_FREQUENCY_96K
#define AUDIO_FREQUENCY_96K       (uint32_t)96000U
#endif
#ifndef AUDIO_FREQUENCY_88K
#define AUDIO_FREQUENCY_88K       (uint32_t)88200U
#endif
#ifndef AUDIO_FREQUENCY_48K
#define AUDIO_FREQUENCY_48K       (uint32_t)48000U
#endif
#ifndef AUDIO_FREQUENCY_44K  
#define AUDIO_FREQUENCY_44K       (uint32_t)44100U
#endif
#ifndef AUDIO_FREQUENCY_32K
#define AUDIO_FREQUENCY_32K       (uint32_t)32000U
#endif
#ifndef AUDIO_FREQUENCY_22K
#define AUDIO_FREQUENCY_22K       (uint32_t)22050U
#endif
#ifndef AUDIO_FREQUENCY_16K
#define AUDIO_FREQUENCY_16K       (uint32_t)16000U
#endif
#ifndef AUDIO_FREQUENCY_11K
#define AUDIO_FREQUENCY_11K       (uint32_t)11025U
#endif
#ifndef AUDIO_FREQUENCY_8K
#define AUDIO_FREQUENCY_8K         (uint32_t)8000U 
#endif
  
  /* AUDIO RESOLUTION */   
#ifndef AUDIO_RESOLUTION_16b
#define AUDIO_RESOLUTION_16b                16U
#endif
#ifndef AUDIO_RESOLUTION_24b
#define AUDIO_RESOLUTION_24b                24U
#endif
#ifndef AUDIO_RESOLUTION_32b
#define AUDIO_RESOLUTION_32b                32U
#endif
  
#define AUDIO_OUT_INSTANCES_NBR            2U
  
  /* Audio Mute state */
#define CCA01M1_AUDIO_MUTE_DISABLED             0U
#define CCA01M1_AUDIO_MUTE_ENABLED              1U
  
  /* Audio Out states */
#define AUDIO_OUT_STATE_RESET               0U
#define AUDIO_OUT_STATE_PLAYING             1U
#define AUDIO_OUT_STATE_STOP                2U
#define AUDIO_OUT_STATE_PAUSE               3U
  
  /* Codec commands */
#define CODEC_PDWN_SW                       1U
#define CODEC_MUTE_ON                       1U
#define CODEC_MUTE_OFF                      0U  

  
#ifdef USE_STM32L4XX_NUCLEO   /* Peripheral defines valid only for NUCLEO-L476RG */

  /* DEVICE 1 */
  /* SAI peripheral configuration defines */  
#define I2S_STANDARD                  			I2S_STANDARD_PHILLIPS  
#define AUDIO_OUT1_SAI_INSTANCE                  SAI2_Block_A
#define AUDIO_OUT1_SAI_CLK_ENABLE()             __HAL_RCC_SAI2_CLK_ENABLE()
#define AUDIO_OUT1_SAI_CLK_DISABLE()             __SAI2_CLK_DISABLE()  
#define AUDIO_OUT1_SAI_SCK_SD_WS_MCK_AF         GPIO_AF13_SAI2
#define AUDIO_OUT1_SAI_SD_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUDIO_OUT1_SAI_SCK_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUDIO_OUT1_SAI_MCK_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define AUDIO_OUT1_SAI_FS_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUDIO_OUT1_SAI_FS_PIN                   GPIO_PIN_12
#define AUDIO_OUT1_SAI_SCK_PIN                  GPIO_PIN_13
#define AUDIO_OUT1_SAI_SD_PIN                   GPIO_PIN_15
#define AUDIO_OUT1_SAI_MCK_PIN                  GPIO_PIN_6
#define AUDIO_OUT1_SAI_SCK_GPIO_PORT            GPIOB
#define AUDIO_OUT1_SAI_SD_GPIO_PORT             GPIOB
#define AUDIO_OUT1_SAI_FS_GPIO_PORT             GPIOB
#define AUDIO_OUT1_SAI_MCK_GPIO_PORT            GPIOC    
  /* SAI DMA Stream tx definitions */
#define AUDIO_OUT1_SAI_DMAx_CLK_ENABLE()        __HAL_RCC_DMA2_CLK_ENABLE()
#define AUDIO_OUT1_SAI_DMAx_INSTANCE            DMA2_Channel3
#define AUDIO_OUT1_SAI_DMAx_REQUEST             DMA_REQUEST_1
#define AUDIO_OUT1_SAI_DMAx_IRQ                 DMA2_Channel3_IRQn
#define AUDIO_OUT1_SAI_DMAx_PERIPH_DATA_SIZE    DMA_PDATAALIGN_HALFWORD
#define AUDIO_OUT1_SAI_DMAx_MEM_DATA_SIZE       DMA_MDATAALIGN_HALFWORD 
#define AUDIO_OUT1_IRQHandler                 	DMA2_Channel3_IRQHandler
  /* Reset Pin definitions */
#define AUDIO_OUT1_RST_GPIO_PORT                GPIOA
#define AUDIO_OUT1_RST_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
#define AUDIO_OUT1_RST_GPIO_CLK_DISABLE()       __GPIOA_CLK_DISABLE()
#define AUDIO_OUT1_RST_PIN                      GPIO_PIN_10  
  /* PD Pin definitions */
#define AUDIO_OUT1_PD_GPIO_PORT           		GPIOA
#define AUDIO_OUT1_PD_GPIO_CLK_ENABLE()   		__GPIOA_CLK_ENABLE()
#define AUDIO_OUT1_PD_GPIO_CLK_DISABLE()  		__GPIOA_CLK_DISABLE()
#define AUDIO_OUT1_PD_PIN                		GPIO_PIN_0
  /* DEVICE 2 */
  /* SAI peripheral configuration defines */  
#define AUDIO_OUT2_SAI_INSTANCE                 SAI2_Block_B
#define AUDIO_OUT2_SAI_CLK_ENABLE()             __SAI2_CLK_ENABLE()
#define AUDIO_OUT2_SAI_CLK_DISABLE()            __SAI2_CLK_DISABLE()
#define AUDIO_OUT2_SAI_SCK_SD_WS_MCK_AF         GPIO_AF13_SAI2                     
#define AUDIO_OUT2_SAI_SD_CLK_ENABLE()          __GPIOC_CLK_ENABLE()
#define AUDIO_OUT2_SAI_SCK_CLK_ENABLE()         __GPIOC_CLK_ENABLE()
#define AUDIO_OUT2_SAI_MCK_CLK_ENABLE()         __GPIOC_CLK_ENABLE()
#define AUDIO_OUT2_SAI_FS_CLK_ENABLE()          __GPIOA_CLK_ENABLE()                     
#define AUDIO_OUT2_SAI_FS_PIN                   GPIO_PIN_15  
#define AUDIO_OUT2_SAI_SCK_PIN                  GPIO_PIN_10
#define AUDIO_OUT2_SAI_SD_PIN                   GPIO_PIN_12  
#define AUDIO_OUT2_SAI_MCK_PIN                  GPIO_PIN_7                     
#define AUDIO_OUT2_SAI_SCK_GPIO_PORT            GPIOC
#define AUDIO_OUT2_SAI_SD_GPIO_PORT             GPIOC                     
#define AUDIO_OUT2_SAI_FS_GPIO_PORT             GPIOA
#define AUDIO_OUT2_SAI_MCK_GPIO_PORT            GPIOC  
  /* SAI DMA Stream tx definitions */
#define AUDIO_OUT2_SAI_DMAx_CLK_ENABLE()        __HAL_RCC_DMA2_CLK_ENABLE()
#define AUDIO_OUT2_SAI_DMAx_INSTANCE            DMA2_Channel4
#define AUDIO_OUT2_SAI_DMAx_REQUEST             DMA_REQUEST_1
#define AUDIO_OUT2_SAI_DMAx_IRQ                 DMA2_Channel4_IRQn
#define AUDIO_OUT2_SAI_DMAx_PERIPH_DATA_SIZE    DMA_PDATAALIGN_HALFWORD
#define AUDIO_OUT2_SAI_DMAx_MEM_DATA_SIZE       DMA_MDATAALIGN_HALFWORD  
#define AUDIO_OUT2_IRQHandler                 	DMA2_Channel4_IRQHandler    
  /* Reset Pin definitions */  
#define AUDIO_OUT2_RST_GPIO_PORT                GPIOA
#define AUDIO_OUT2_RST_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
#define AUDIO_OUT2_RST_GPIO_CLK_DISABLE()       __GPIOA_CLK_DISABLE()
#define AUDIO_OUT2_RST_PIN                      GPIO_PIN_8  
  /* PD Pin definitions */
  
#define AUDIO_OUT2_PD_GPIO_PORT                 GPIOB
#define AUDIO_OUT2_PD_GPIO_CLK_ENABLE()         __GPIOB_CLK_ENABLE()
#define AUDIO_OUT2_PD_GPIO_CLK_DISABLE()        __GPIOB_CLK_DISABLE()
#define AUDIO_OUT2_PD_PIN                       GPIO_PIN_0

#else /* Peripheral defines for NUCLEO-L0, NUCLEO-F0, NUCLEO-F4 and NUCLEO-F7 */

  /* DEVICE 1 */ 
  /* I2S peripheral configuration defines */
#define I2S_STANDARD                  			I2S_STANDARD_PHILLIPS
#define AUDIO_OUT1_I2S_INSTANCE                 SPI2
#define AUDIO_OUT1_I2S_CLK_ENABLE()             __SPI2_CLK_ENABLE()
#define AUDIO_OUT1_I2S_CLK_DISABLE()            __SPI2_CLK_DISABLE()

#if (defined (USE_STM32F0XX_NUCLEO) || defined(USE_STM32L0XX_NUCLEO))
#define AUDIO_OUT1_I2S_SCK_AF         	        GPIO_AF0_SPI2
#define AUDIO_OUT1_I2S_WS_AF         		GPIO_AF0_SPI2
#define AUDIO_OUT1_I2S_SD_AF         		GPIO_AF0_SPI2
#else
#define AUDIO_OUT1_I2S_SCK_AF         	        GPIO_AF5_SPI2
#define AUDIO_OUT1_I2S_WS_AF         		GPIO_AF5_SPI2
#define AUDIO_OUT1_I2S_SD_AF         		GPIO_AF5_SPI2
#endif

#ifdef USE_STM32L0XX_NUCLEO
#define AUDIO_OUT1_I2S_MCK_AF         		GPIO_AF2_SPI2
#else
#define AUDIO_OUT1_I2S_MCK_AF         		GPIO_AF5_SPI2
#endif

#define AUDIO_OUT1_I2S_SD_CLK_ENABLE()          __GPIOB_CLK_ENABLE()
#define AUDIO_OUT1_I2S_SCK_CLK_ENABLE()         __GPIOB_CLK_ENABLE()
#define AUDIO_OUT1_I2S_MCK_CLK_ENABLE()         __GPIOC_CLK_ENABLE()
#define AUDIO_OUT1_I2S_WS_CLK_ENABLE()          __GPIOB_CLK_ENABLE()
#define AUDIO_OUT1_I2S_WS_PIN                   GPIO_PIN_12
#define AUDIO_OUT1_I2S_SCK_PIN                  GPIO_PIN_13
#define AUDIO_OUT1_I2S_SD_PIN                   GPIO_PIN_15

#if (defined (USE_STM32F0XX_NUCLEO) || defined(USE_STM32L0XX_NUCLEO))
#define AUDIO_OUT1_I2S_MCK_PIN                  GPIO_PIN_2
#else
#define AUDIO_OUT1_I2S_MCK_PIN                  GPIO_PIN_6
#endif 

#define AUDIO_OUT1_I2S_SCK_GPIO_PORT            GPIOB
#define AUDIO_OUT1_I2S_SD_GPIO_PORT             GPIOB
#define AUDIO_OUT1_I2S_WS_GPIO_PORT             GPIOB
#define AUDIO_OUT1_I2S_MCK_GPIO_PORT            GPIOC   

  /* I2S DMA Stream Tx definitions */
#define AUDIO_OUT1_I2S_DMAx_CLK_ENABLE()        __DMA1_CLK_ENABLE()
#define AUDIO_OUT1_I2S_DMAx_PERIPH_DATA_SIZE    DMA_PDATAALIGN_HALFWORD
#define AUDIO_OUT1_I2S_DMAx_MEM_DATA_SIZE       DMA_MDATAALIGN_HALFWORD 

#if (defined (USE_STM32F0XX_NUCLEO) || defined(USE_STM32L0XX_NUCLEO))
#define AUDIO_OUT1_I2S_DMAx_STREAM              DMA1_Channel5
#define AUDIO_OUT1_I2S_DMAx_IRQ                 DMA1_Channel4_5_6_7_IRQn
#define AUDIO_OUT1_IRQHandler                 	DMA1_Channel4_5_6_7_IRQHandler 
#else
#define AUDIO_OUT1_I2S_DMAx_STREAM              DMA1_Stream4
#define AUDIO_OUT1_I2S_DMAx_CHANNEL             DMA_CHANNEL_0
#define AUDIO_OUT1_I2S_DMAx_IRQ                 DMA1_Stream4_IRQn
#define AUDIO_OUT1_IRQHandler                 	DMA1_Stream4_IRQHandler
#endif

  /* Reset Pin definitions */
#define AUDIO_OUT1_RST_GPIO_PORT                GPIOA
#define AUDIO_OUT1_RST_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
#define AUDIO_OUT1_RST_GPIO_CLK_DISABLE()       __GPIOA_CLK_DISABLE()
#define AUDIO_OUT1_RST_PIN                      GPIO_PIN_10  
  /* PD Pin definitions */
#define AUDIO_OUT1_PD_GPIO_PORT           		GPIOA
#define AUDIO_OUT1_PD_GPIO_CLK_ENABLE()   		__GPIOA_CLK_ENABLE()
#define AUDIO_OUT1_PD_GPIO_CLK_DISABLE()  		__GPIOA_CLK_DISABLE()
#define AUDIO_OUT1_PD_PIN                		GPIO_PIN_0
  
  /* DEVICE 2 */ 
  /* I2S peripheral configuration defines */
#ifdef USE_STM32F0XX_NUCLEO 
#define AUDIO_OUT2_I2S_INSTANCE                  SPI1
#define AUDIO_OUT2_I2S_CLK_ENABLE()             __SPI1_CLK_ENABLE()
#define AUDIO_OUT2_I2S_CLK_DISABLE()            __SPI1_CLK_DISABLE()
#define AUDIO_OUT2_I2S_SCK_AF         	        GPIO_AF0_SPI1
#define AUDIO_OUT2_I2S_SD_AF         		GPIO_AF0_SPI1
#define AUDIO_OUT2_I2S_WS_AF         		GPIO_AF0_SPI1
#define AUDIO_OUT2_I2S_MCK_AF         		GPIO_AF0_SPI1
#define AUDIO_OUT2_I2S_SD_CLK_ENABLE()          __GPIOA_CLK_ENABLE()
#define AUDIO_OUT2_I2S_SCK_CLK_ENABLE()         __GPIOA_CLK_ENABLE()
#define AUDIO_OUT2_I2S_MCK_CLK_ENABLE()         __GPIOA_CLK_ENABLE()
#define AUDIO_OUT2_I2S_WS_CLK_ENABLE()          __GPIOA_CLK_ENABLE()
#define AUDIO_OUT2_I2S_WS_PIN                   GPIO_PIN_4
#define AUDIO_OUT2_I2S_SCK_PIN                  GPIO_PIN_5
#define AUDIO_OUT2_I2S_SD_PIN                   GPIO_PIN_7
#define AUDIO_OUT2_I2S_MCK_PIN                  GPIO_PIN_6
#define AUDIO_OUT2_I2S_SCK_GPIO_PORT            GPIOA
#define AUDIO_OUT2_I2S_SD_GPIO_PORT             GPIOA
#define AUDIO_OUT2_I2S_WS_GPIO_PORT             GPIOA
#define AUDIO_OUT2_I2S_MCK_GPIO_PORT            GPIOA    
  /* I2S DMA Stream Tx definitions */
#define AUDIO_OUT2_I2S_DMAx_CLK_ENABLE()        __DMA1_CLK_ENABLE()
#define AUDIO_OUT2_I2S_DMAx_STREAM              DMA1_Channel3
#define AUDIO_OUT2_I2S_DMAx_IRQ                 DMA1_Channel2_3_IRQn
#define AUDIO_OUT2_I2S_DMAx_PERIPH_DATA_SIZE    DMA_PDATAALIGN_HALFWORD
#define AUDIO_OUT2_I2S_DMAx_MEM_DATA_SIZE       DMA_MDATAALIGN_HALFWORD 
#define AUDIO_OUT2_IRQHandler                 	DMA1_Channel2_3_IRQHandler 

#else
#define AUDIO_OUT2_I2S_INSTANCE                  SPI3
#define AUDIO_OUT2_I2S_CLK_ENABLE()             __SPI3_CLK_ENABLE()
#define AUDIO_OUT2_I2S_CLK_DISABLE()             __SPI3_CLK_DISABLE()
#define AUDIO_OUT2_I2S_SCK_AF         	        GPIO_AF6_SPI3
#define AUDIO_OUT2_I2S_SD_AF         		GPIO_AF6_SPI3
#define AUDIO_OUT2_I2S_WS_AF         		GPIO_AF6_SPI3
#define AUDIO_OUT2_I2S_MCK_AF         		GPIO_AF6_SPI3                    
#define AUDIO_OUT2_I2S_SD_CLK_ENABLE()          __GPIOC_CLK_ENABLE()
#define AUDIO_OUT2_I2S_SCK_CLK_ENABLE()         __GPIOC_CLK_ENABLE()
#define AUDIO_OUT2_I2S_MCK_CLK_ENABLE()         __GPIOC_CLK_ENABLE()
#define AUDIO_OUT2_I2S_WS_CLK_ENABLE()          __GPIOA_CLK_ENABLE()                     
#define AUDIO_OUT2_I2S_WS_PIN                   GPIO_PIN_4  
#define AUDIO_OUT2_I2S_SCK_PIN                  GPIO_PIN_10
#define AUDIO_OUT2_I2S_SD_PIN                   GPIO_PIN_12  
#define AUDIO_OUT2_I2S_MCK_PIN                  GPIO_PIN_7                     
#define AUDIO_OUT2_I2S_SCK_GPIO_PORT            GPIOC
#define AUDIO_OUT2_I2S_SD_GPIO_PORT             GPIOC                     
#define AUDIO_OUT2_I2S_WS_GPIO_PORT             GPIOA
#define AUDIO_OUT2_I2S_MCK_GPIO_PORT            GPIOC    
  /* I2S DMA Stream Tx definitions */
#define AUDIO_OUT2_I2S_DMAx_CLK_ENABLE()        __DMA1_CLK_ENABLE()
#define AUDIO_OUT2_I2S_DMAx_STREAM              DMA1_Stream7
#define AUDIO_OUT2_I2S_DMAx_CHANNEL             DMA_CHANNEL_0
#define AUDIO_OUT2_I2S_DMAx_IRQ                 DMA1_Stream7_IRQn
#define AUDIO_OUT2_I2S_DMAx_PERIPH_DATA_SIZE    DMA_PDATAALIGN_HALFWORD
#define AUDIO_OUT2_I2S_DMAx_MEM_DATA_SIZE       DMA_MDATAALIGN_HALFWORD
#define  AUDIO_OUT2_IRQHandler                  DMA1_Stream7_IRQHandler
#endif

  /* Reset Pin definitions */
#define AUDIO_OUT2_RST_GPIO_PORT                GPIOA
#define AUDIO_OUT2_RST_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
#define AUDIO_OUT2_RST_GPIO_CLK_DISABLE()       __GPIOA_CLK_DISABLE()
#define AUDIO_OUT2_RST_PIN                      GPIO_PIN_8  
  /* PD Pin definitions */  
#define AUDIO_OUT2_PD_GPIO_PORT           		GPIOB
#define AUDIO_OUT2_PD_GPIO_CLK_ENABLE()   		__GPIOB_CLK_ENABLE()
#define AUDIO_OUT2_PD_GPIO_CLK_DISABLE()  		__GPIOB_CLK_DISABLE()
#define AUDIO_OUT2_PD_PIN                		GPIO_PIN_0
  
#endif

  /* Select the interrupt preemption priority and subpriority for the IT/DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO            6   /* Select the preemption priority level(0 is the highest) */
#define DMA_MAX_SZE                     0xFFFF
#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
  /* Audio status definition */     
#ifndef AUDIO_OK
#define AUDIO_OK                                ((uint8_t)0)
#endif    
#ifndef AUDIO_ERROR
#define AUDIO_ERROR                             ((uint8_t)1)
#endif   
#ifndef AUDIO_TIMEOUT
#define AUDIO_TIMEOUT                           ((uint8_t)2)
#endif   
  
#ifndef POS_VAL
#define POS_VAL(VAL)                  (POSITION_VAL(VAL) - 4U)
#endif
#define VOLUME_OUT_CONVERT(Volume)    (((Volume) > 100)? 63:((uint8_t)(((Volume) * 63) / 100)))
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 239) / 100)))
/**
* @}
*/


/** @defgroup STAMPINO_F401_AUDIO_Exported_Macros AUDIO_Exported_Macros
* @{
*/

/** @defgroup  X_NUCLEO_CCA01M1_AUDIO_Exported_Functions 
* @{
*/

/** @defgroup  X_NUCLEO_CCA01M1_AUDIO_Exported_Functions Exported Functions 
* @{
*/

int32_t CCA01M1_AUDIO_OUT_Init(uint32_t Instance, CCA01M1_AUDIO_Init_t* AudioInit);    
int32_t CCA01M1_AUDIO_OUT_DeInit(uint32_t Instance);
int32_t CCA01M1_AUDIO_OUT_Play(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes);
int32_t CCA01M1_AUDIO_OUT_Pause(uint32_t Instance);
int32_t CCA01M1_AUDIO_OUT_Resume(uint32_t Instance);
int32_t CCA01M1_AUDIO_OUT_Stop(uint32_t Instance);
int32_t CCA01M1_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t CCA01M1_AUDIO_OUT_Mute(uint32_t Instance);
int32_t CCA01M1_AUDIO_OUT_UnMute(uint32_t Instance);
int32_t CCA01M1_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute);
int32_t CCA01M1_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device);
int32_t CCA01M1_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t CCA01M1_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t CCA01M1_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);
int32_t CCA01M1_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t CCA01M1_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);
int32_t CCA01M1_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t CCA01M1_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t CCA01M1_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t CCA01M1_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function is called when the requested data has been completely transferred.*/
void    CCA01M1_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance);

/* This function is called when half of the requested buffer has been transferred. */
void    CCA01M1_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance);

/* This function is called when an Interrupt due to transfer error on or peripheral
error occurs. */
void    CCA01M1_AUDIO_OUT_Error_CallBack(void);

int32_t CCA01M1_AUDIO_OUT_Reset(uint32_t Instance);

#ifdef USE_STM32L4XX_NUCLEO
HAL_StatusTypeDef MX_SAI_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t SampleRate);
HAL_StatusTypeDef MX_SAI_Init(SAI_HandleTypeDef *hsai, MX_SAI_Config *MXConfig);
#else
HAL_StatusTypeDef MX_I2S_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t SampleRate);
HAL_StatusTypeDef MX_I2S_OUT_Init(I2S_HandleTypeDef *hi2s, MX_I2S_OUT_Config *MXConfig);
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

#ifdef __cplusplus
}
#endif

#endif /* CCA01M1_AUDIO_H */

