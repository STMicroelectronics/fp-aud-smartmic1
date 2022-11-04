/**
******************************************************************************
* @file    BlueCoin_audio.h
* @author  SRA - Central Labs
* @version v2.2.3
* @date    10-Feb-2022
* @brief   This file contains the common defines and functions prototypes for
*          the BlueCoin_audio.c driver.
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLUECOIN_AUDIO_H
#define BLUECOIN_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "BlueCoin_conf.h"
#include "audio.h"
#include "PCM1774.h"
#include <stdlib.h>
/* Include PDM to PCM lib header file */
#include "pdm2pcm_glo.h"



/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup BLUECOIN
  * @{
  */
    
/** @addtogroup BLUECOIN_AUDIO 
  * @{
  */
   
/** @defgroup X-NUCLEO-CCA02M1_AUDIO_Exported_Variables
 * @{
 */

extern I2S_HandleTypeDef                 hAudioInI2s;

#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define HTONS(A)  ((((A) & (uint16_t)0xff00) >> 8) | (((A) & (uint16_t)0x00ff) << 8))

/**
 * @}
 */   

/** @defgroup BLUECOIN_AUDIO_Exported_Types BLUECOIN_AUDIO Exported Types
  * @{
  */

typedef enum
{
  CODEX_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  PCM1774_0 = 0                   /* . */  
} CODEX_ID_t;

typedef struct {
	int32_t Z; 
	int32_t oldOut; 
	int32_t oldIn; 
}HP_FilterState_TypeDef;  
  
typedef struct
{                                   
  uint32_t                    Device;                                           
  uint32_t                    SampleRate;                                         
  uint32_t                    BitsPerSample;                                          
  uint32_t                    ChannelsNbr;                                         
  uint32_t                    Volume;
}BSP_AUDIO_Init_t;

typedef struct
{
  uint32_t                    Instance;            /* Audio IN instance              */  
  uint32_t                    Device;              /* Audio IN device to be used     */ 
  uint32_t                    SampleRate;          /* Audio IN Sample rate           */
  uint32_t                    BitsPerSample;       /* Audio IN Sample resolution     */
  uint32_t                    ChannelsNbr;         /* Audio IN number of channel     */
  uint16_t                    *pBuff;              /* Audio IN record buffer         */
  uint8_t                     **pMultiBuff;        /* Audio IN multi-buffer          */
  uint32_t                    Size;                /* Audio IN record buffer size    */
  uint32_t                    Volume;              /* Audio IN volume                */
  uint32_t                    State;               /* Audio IN State                 */
  uint32_t                    IsMultiBuff;         /* Audio IN multi-buffer usage    */
  uint32_t                    IsMspCallbacksValid; /* Is Msp Callbacks registred     */
  HP_FilterState_TypeDef 	  HP_Filters[4];       /*!< HP filter state for each channel*/
  uint32_t DecimationFactor;
}AUDIO_IN_Ctx_t;
  
typedef struct
{
  uint32_t                    Instance;            /* Audio OUT instance              */  
  uint32_t                    Device;              /* Audio OUT device to be used     */ 
  uint32_t                    SampleRate;          /* Audio OUT Sample rate           */
  uint32_t                    BitsPerSample;       /* Audio OUT Sample Bit Per Sample */
  uint32_t                    Volume;              /* Audio OUT volume                */
  uint32_t                    ChannelsNbr;         /* Audio OUT number of channel     */
  uint32_t                    IsMute;              /* Mute state                      */   
  uint32_t                    State;               /* Audio OUT State                 */
  uint32_t                    IsMspCallbacksValid; /* Is Msp Callbacks registred      */ 
}AUDIO_OUT_Ctx_t;

typedef struct
{
  uint32_t Mode;            
  uint32_t Standard;       
  uint32_t DataFormat;     
  uint32_t MCLKOutput;      
  uint32_t AudioFreq;       
  uint32_t CPOL;            
  uint32_t ClockSource;     
  uint32_t FullDuplexMode; 
}MX_I2S_Config;

typedef struct
{
  uint32_t Mode;
  uint32_t Direction;  
  uint32_t DataSize;    
  uint32_t CLKPolarity; 
  uint32_t CLKPhase;   
  uint32_t NSS;        
  uint32_t BaudRatePrescaler;
  uint32_t FirstBit;       
  uint32_t TIMode;         
  uint32_t CRCCalculation; 
  uint32_t CRCPolynomial;	
}MX_SPI_Config;

typedef struct
{
  uint32_t AudioFrequency;
  uint32_t AudioMode;
  uint32_t DataSize;
  uint32_t ClockSource;
  uint32_t MonoStereoMode;
  uint32_t ClockStrobing;
  uint32_t Synchro;
  uint32_t OutputDrive;
  uint32_t SynchroExt;
  uint32_t FrameLength;
  uint32_t ActiveFrameLength;
  uint32_t SlotActive; 
  uint32_t Mckdiv;
}MX_SAI_Config;

/**
  * @}
  */ 

/** @defgroup BLUECOIN_AUDIO_Exported_Constants BLUECOIN_AUDIO Exported Constants
  * @{
  */

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


#define AUDIO_OUT_INSTANCES_NBR            1U

/* Select the interrupt preemption priority for the DMA interrupt */
#define BSP_AUDIO_OUT_IT_PRIORITY       5U

/* Audio Mute state */
#define BSP_AUDIO_MUTE_DISABLED             0U
#define BSP_AUDIO_MUTE_ENABLED              1U

/* Audio Out states */
#define AUDIO_OUT_STATE_RESET               0U
#define AUDIO_OUT_STATE_PLAYING             1U
#define AUDIO_OUT_STATE_STOP                2U
#define AUDIO_OUT_STATE_PAUSE               3U

/* Codec commands */
#define CODEC_PDWN_SW                       1U
#define CODEC_MUTE_ON                       1U
#define CODEC_MUTE_OFF                      0U
    
/*------------------------------------------------------------------------------
                        AUDIO OUT defines parameters
------------------------------------------------------------------------------*/    
/* SAI peripheral configuration defines */
#define AUDIO_SAIx                           SAI1_Block_A
#define AUDIO_SAIx_CLK_ENABLE()              __SAI1_CLK_ENABLE()
#define AUDIO_SAIx_CLK_DISABLE()              __SAI1_CLK_DISABLE()
#define AUDIO_SAIx_MCLK_SCK_SD_FS_AF         GPIO_AF6_SAI1

#define AUDIO_SAIx_SCK_ENABLE()              __GPIOB_CLK_ENABLE()
#define AUDIO_SAIx_SD_ENABLE()               __GPIOB_CLK_ENABLE()
#define AUDIO_SAIx_MCLK_ENABLE()             __GPIOE_CLK_ENABLE()
#define AUDIO_SAIx_FS_ENABLE()               __GPIOE_CLK_ENABLE()
#define AUDIO_SAIx_SCK_GPIO_PORT             GPIOB
#define AUDIO_SAIx_SD_GPIO_PORT              GPIOB
#define AUDIO_SAIx_MCLK_GPIO_PORT            GPIOE
#define AUDIO_SAIx_FS_GPIO_PORT              GPIOE
#define AUDIO_SAIx_SCK_PIN                   GPIO_PIN_10
#define AUDIO_SAIx_SD_PIN                    GPIO_PIN_2
#define AUDIO_SAIx_FS_PIN                    GPIO_PIN_4
#define AUDIO_SAIx_MCLK_PIN                  GPIO_PIN_2

/* SAI DMA Stream definitions */
#define AUDIO_SAIx_DMAx_CLK_ENABLE()         __DMA2_CLK_ENABLE()
#define AUDIO_SAIx_DMAx_STREAM               DMA2_Stream1
#define AUDIO_SAIx_DMAx_CHANNEL              DMA_CHANNEL_0
#define AUDIO_SAIx_DMAx_IRQ                  DMA2_Stream1_IRQn
#define AUDIO_SAIx_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define AUDIO_SAIx_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_HALFWORD
#define DMA_MAX_SZE                          (uint32_t)0xFFFF

#define AUDIO_SAIx_DMAx_IRQHandler           DMA2_Stream1_IRQHandler
  
  /* Select the interrupt preemption priority for the DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO           5U   /* Select the preemption priority level(0 is the highest) */   


/*------------------------------------------------------------------------------
                        AUDIO IN defines parameters
------------------------------------------------------------------------------*/ 
  /* I2S Configuration defines */
#define AUDIO_IN_I2S_INSTANCE                                    SPI2
#define AUDIO_IN_I2S_CLK_ENABLE()                                __SPI2_CLK_ENABLE()
#define AUDIO_IN_I2S_SCK_PIN                                     GPIO_PIN_13
#define AUDIO_IN_I2S_SCK_GPIO_PORT                               GPIOB
#define AUDIO_IN_I2S_SCK_GPIO_CLK_ENABLE()                       __GPIOB_CLK_ENABLE()

#define AUDIO_IN_1CH_I2S_SCK_PIN                                     GPIO_PIN_7
#define AUDIO_IN_1CH_I2S_SCK_GPIO_PORT                               GPIOC
#define AUDIO_IN_1CH_I2S_SCK_GPIO_CLK_ENABLE()                       __GPIOC_CLK_ENABLE()

#define AUDIO_IN_I2S_SCK_AF                                      GPIO_AF5_SPI2
#define AUDIO_IN_I2S_MOSI_PIN                                    GPIO_PIN_3
#define AUDIO_IN_I2S_MOSI_GPIO_PORT                              GPIOC
#define AUDIO_IN_I2S_MOSI_GPIO_CLK_ENABLE()                      __GPIOC_CLK_ENABLE()
#define AUDIO_IN_I2S_MOSI_AF                                     GPIO_AF5_SPI2
  
  /* I2S DMA definitions */
#define AUDIO_IN_I2S_DMAx_CLK_ENABLE()                          __DMA1_CLK_ENABLE()
#define AUDIO_IN_I2S_DMAx_STREAM                                DMA1_Stream3
#define AUDIO_IN_I2S_DMAx_CHANNEL                               DMA_CHANNEL_0
#define AUDIO_IN_I2S_DMAx_IRQ                                   DMA1_Stream3_IRQn
#define AUDIO_IN_I2S_DMAx_PERIPH_DATA_SIZE                      DMA_PDATAALIGN_HALFWORD
#define AUDIO_IN_I2S_DMAx_MEM_DATA_SIZE                         DMA_MDATAALIGN_HALFWORD  
#define AUDIO_IN_I2S_IRQHandler                                 DMA1_Stream3_IRQHandler                                                 


#define AUDIO_IN_SPI_INSTANCE                                            SPI3
#define AUDIO_IN_SPI_CLK_ENABLE()                               __SPI3_CLK_ENABLE()
#define AUDIO_IN_SPI_SCK_GPIO_CLK_ENABLE()                      __GPIOC_CLK_ENABLE()
#define AUDIO_IN_SPI_MISO_GPIO_CLK_ENABLE()                     __GPIOC_CLK_ENABLE() 
#define AUDIO_IN_SPI_MOSI_GPIO_CLK_ENABLE()                     __GPIOC_CLK_ENABLE()   
#define AUDIO_IN_SPI_FORCE_RESET()                              __SPI3_FORCE_RESET()
#define AUDIO_IN_SPI_RELEASE_RESET()                            __SPI3_RELEASE_RESET()
#define AUDIO_IN_SPI_SCK_PIN                                    GPIO_PIN_10
#define AUDIO_IN_SPI_SCK_GPIO_PORT                              GPIOC
#define AUDIO_IN_SPI_SCK_AF                                     GPIO_AF6_SPI3
#define AUDIO_IN_SPI_MOSI_PIN                                   GPIO_PIN_12
#define AUDIO_IN_SPI_MOSI_GPIO_PORT                             GPIOC
#define AUDIO_IN_SPI_MOSI_AF                                    GPIO_AF6_SPI3
  
  /* SPI DMA definitions */
#define AUDIO_IN_SPI_DMAx_CLK_ENABLE()                          __DMA1_CLK_ENABLE()
#define AUDIO_IN_SPI_RX_DMA_CHANNEL                             DMA_CHANNEL_0
#define AUDIO_IN_SPI_RX_DMA_STREAM                              DMA1_Stream2 
#define AUDIO_IN_SPI_DMA_RX_IRQn                                DMA2_Stream2_IRQn
#define AUDIO_IN_SPI_DMA_RX_IRQHandler                          DMA2_Stream2_IRQHandler
  


/* AUDIO TIMER definitions */
#define AUDIO_IN_TIMER                                     TIM3
#define AUDIO_IN_TIMER_CLK_ENABLE()                        __TIM3_CLK_ENABLE()  
#define AUDIO_IN_TIMER_CHOUT_AF                            GPIO_AF2_TIM3
#define AUDIO_IN_TIMER_CHOUT_PIN                           GPIO_PIN_5
#define AUDIO_IN_TIMER_CHOUT_GPIO_PORT                     GPIOB
#define AUDIO_IN_TIMER_CHOUT_GPIO_PORT_CLK_ENABLE()        __GPIOB_CLK_ENABLE()  
#define AUDIO_IN_TIMER_CHIN_AF                             GPIO_AF2_TIM3
#define AUDIO_IN_TIMER_CHIN_PIN                            GPIO_PIN_4
#define AUDIO_IN_TIMER_CHIN_GPIO_PORT                      GPIOB
#define AUDIO_IN_TIMER_CHIN_GPIO_PORT_CLK_ENABLE()         __GPIOB_CLK_ENABLE() 
/* Audio In devices */ 

/* MP34DT01TR digital microphone on PCB top side */
#define AUDIO_IN_DIGITAL_MIC1      0x10U
#define AUDIO_IN_DIGITAL_MIC2      0x20U
#define AUDIO_IN_DIGITAL_MIC3      0x40U
#define AUDIO_IN_DIGITAL_MIC4      0x80U
#define AUDIO_IN_DIGITAL_MIC_LAST  AUDIO_IN_DIGITAL_MIC4
#define AUDIO_IN_DIGITAL_MIC       (AUDIO_IN_DIGITAL_MIC1 | AUDIO_IN_DIGITAL_MIC2 | AUDIO_IN_DIGITAL_MIC3 | AUDIO_IN_DIGITAL_MIC4)
#define DFSDM_MIC_NUMBER           AUDIO_CHANNELS

/* Default Audio IN internal buffer size */   
#define DEFAULT_AUDIO_IN_BUFFER_SIZE        (AUDIO_SAMPLING_FREQUENCY/1000)*2

/* Buffer size defines for F4 and F7*/

#define CHANNEL_DEMUX_MASK                    	0x55U
    
#define MAX_MIC_FREQ                 	  3072  /*KHz*/
#define MAX_AUDIO_IN_CHANNEL_NBR_PER_IF   2 
#define MAX_AUDIO_IN_CHANNEL_NBR_TOTAL    4 

/*Number of millisecond of audio at each DMA interrupt*/
#define N_MS_PER_INTERRUPT               (1U)
    
/*BSP internal buffer size in half words (16 bits)*/  
#define PDM_INTERNAL_BUFFER_SIZE_I2S          ((MAX_MIC_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_PER_IF * N_MS_PER_INTERRUPT)
#if MAX_AUDIO_IN_CHANNEL_NBR_TOTAL > 2
#define PDM_INTERNAL_BUFFER_SIZE_SPI          ((MAX_MIC_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_PER_IF * N_MS_PER_INTERRUPT)
#else
#define PDM_INTERNAL_BUFFER_SIZE_SPI          1
#endif

/* Audio In states */
#define AUDIO_IN_STATE_RESET               0U
#define AUDIO_IN_STATE_RECORDING           1U
#define AUDIO_IN_STATE_STOP                2U
#define AUDIO_IN_STATE_PAUSE               3U

/* Audio In instances number:
   Instance 0 is I2S / SPI path
   Instance 1 is DFSDM path
   Instance 2 is PDM path
 */
#define AUDIO_IN_INSTANCES_NBR             2U
/**
  * @}
  */
   
/** @defgroup BLUECOIN_AUDIO_Exported_Macros BLUECOIN_AUDIO Exported Macros
  * @{
  */
#define POS_VAL(VAL)                  (POSITION_VAL(VAL) - 4U)
#define VOLUME_OUT_CONVERT(Volume)    (((Volume) > 100)? 63:((uint8_t)(((Volume) * 63) / 100)))
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 239) / 100)))
    
/**
  * @}
  */ 
/** @addtogroup BLUECOIN_AUDIO_Exported_Variables
  * @{
  */
/* Recording context */
extern AUDIO_IN_Ctx_t                         AudioInCtx[];
/**
  * @}
  */

/** @defgroup BLUECOIN_AUDIO_Exported_Functions BLUECOIN_AUDIO_IN Exported Functions
  * @{
  */
int32_t BSP_AUDIO_OUT_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit);    
int32_t BSP_AUDIO_OUT_DeInit(uint32_t Instance);

int32_t BSP_AUDIO_OUT_Play(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes);
int32_t BSP_AUDIO_OUT_Pause(uint32_t Instance);
int32_t BSP_AUDIO_OUT_Resume(uint32_t Instance);
int32_t BSP_AUDIO_OUT_Stop(uint32_t Instance);
int32_t BSP_AUDIO_OUT_Mute(uint32_t Instance);
int32_t BSP_AUDIO_OUT_UnMute(uint32_t Instance);
int32_t BSP_AUDIO_OUT_IsMute(uint32_t Instance, uint32_t *IsMute);
int32_t BSP_AUDIO_OUT_SetDevice(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_OUT_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t BSP_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t BSP_AUDIO_OUT_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);
int32_t BSP_AUDIO_OUT_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t BSP_AUDIO_OUT_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);
int32_t BSP_AUDIO_OUT_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t BSP_AUDIO_OUT_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t BSP_AUDIO_OUT_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t BSP_AUDIO_OUT_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t BSP_AUDIO_OUT_GetState(uint32_t Instance, uint32_t *State);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function is called when the requested data has been completely transferred.*/
void    BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance);

/* This function is called when half of the requested buffer has been transferred. */
void    BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void    BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance);

/* These function can be modified in case the current settings need to be changed 
   for specific application needs */
HAL_StatusTypeDef MX_SAI1_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t SampleRate);
HAL_StatusTypeDef MX_SAI1_Block_A_Init(SAI_HandleTypeDef* hsai, MX_SAI_Config *MXConfig);


int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit);    
int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance);
int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes);
int32_t BSP_AUDIO_IN_Stop(uint32_t Instance);
int32_t BSP_AUDIO_IN_Pause(uint32_t Instance);
int32_t BSP_AUDIO_IN_Resume(uint32_t Instance);

int32_t BSP_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes);
int32_t BSP_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device);

int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);                 
int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);                
int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State);

/* Specific PDM recodr APIs */
int32_t BSP_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut);
int32_t BSP_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf);
int32_t BSP_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes);

void BSP_AUDIO_IN_DMA1_Stream4_IRQHandler(void);
void BSP_AUDIO_IN_DMA1_Stream5_IRQHandler(void);
void BSP_AUDIO_IN_DMA1_Stream6_IRQHandler(void);
void BSP_AUDIO_IN_DMA1_Stream7_IRQHandler(void);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
   It is called into this driver when the current buffer is filled to prepare the next
   buffer pointer and its size. */
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance);

/* These function can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
HAL_StatusTypeDef MX_I2S_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t PDM_rate);
HAL_StatusTypeDef MX_I2S_Init(I2S_HandleTypeDef* hi2s, MX_I2S_Config *MXConfig);
HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef* hspi, MX_SPI_Config *MXConfig);

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

#endif /* BLUECOIN_AUDIO_H */
