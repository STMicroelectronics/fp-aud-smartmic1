/**
******************************************************************************
* @file    cca02m2_audio.h
* @author  SRA
* @version v1.1.2
* @date    10-Feb-2022
* @brief   This file contains the common defines and functions prototypes for
*          the cca02m2_audio.c driver.
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
#ifndef CCA02M2_AUDIO_H
#define CCA02M2_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "cca02m2_conf.h"
#include <stdlib.h>

#ifndef USE_STM32L4XX_NUCLEO   
/* Include PDM to PCM lib header file */
#include "pdm2pcm_glo.h"
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
   
/** @defgroup CCA02M2_AUDIO_Exported_Variables CCA02M2_AUDIO_ Exported Variables
  * @{
  */
#ifdef USE_STM32L4XX_NUCLEO
extern DMA_HandleTypeDef               hDmaDfsdm[];
#else

#ifdef USE_STM32WBXX_NUCLEO
extern SAI_HandleTypeDef            hAudioInSai;
#else
extern I2S_HandleTypeDef                 hAudioInI2s;
#endif

#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define HTONS(A)  ((((A) & (uint16_t)0xff00) >> 8) | (((A) & (uint16_t)0x00ff) << 8))
#endif

/**
 * @}
 */   

/** @defgroup CCA02M2_AUDIO_Exported_Types CCA02M2_AUDIO_ Exported Types
  * @{
  */

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
}CCA02M2_AUDIO_Init_t;

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
  uint32_t Mode;            
  uint32_t Standard;       
  uint32_t DataFormat;     
  uint32_t MCLKOutput;      
  uint32_t AudioFreq;       
  uint32_t CPOL;            
  uint32_t ClockSource;     
  uint32_t FullDuplexMode; 
}MX_I2S_IN_Config;

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

#ifdef USE_STM32L4XX_NUCLEO

typedef struct
{
  /* Filter parameters */
  DFSDM_Filter_TypeDef   *FilterInstance;
  uint32_t               RegularTrigger;
  uint32_t               SincOrder;   
  uint32_t               Oversampling;
  /* Channel parameters */
  DFSDM_Channel_TypeDef *ChannelInstance;
  uint32_t              DigitalMicPins;
  uint32_t              DigitalMicType;
  uint32_t              Channel4Filter;
  uint32_t              ClockDivider;
  uint32_t              RightBitShift; 
}MX_DFSDM_Config;  

#if ((USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) || (USE_HAL_SAI_REGISTER_CALLBACKS == 1))
typedef struct
{
#if (USE_HAL_SAI_REGISTER_CALLBACKS == 1)	
  pSAI_CallbackTypeDef            pMspSaiInitCb;
  pSAI_CallbackTypeDef            pMspSaiDeInitCb;
#endif
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)  
  pDFSDM_Filter_CallbackTypeDef   pMspFltrInitCb; 
  pDFSDM_Filter_CallbackTypeDef   pMspFltrDeInitCb;
  pDFSDM_Channel_CallbackTypeDef  pMspChInitCb; 
  pDFSDM_Channel_CallbackTypeDef  pMspChDeInitCb;
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */  
}CCA02M2_AUDIO_IN_Cb_t;
#endif /* ((USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) || (USE_HAL_SAI_REGISTER_CALLBACKS == 1)) */

#endif

/**
  * @}
  */ 

/** @defgroup CCA02M2_AUDIO_Exported_Constants CCA02M2_AUDIO_ Exported Constants
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




/*------------------------------------------------------------------------------
                          USER I2S / SPI defines parameters
 -----------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
                        AUDIO IN defines parameters
------------------------------------------------------------------------------*/ 
#ifdef USE_STM32WBXX_NUCLEO /* defines valid only for STM32WB */

  /* I2S Configuration defines */
#define AUDIO_IN_SAI_INSTANCE                                    SAI1_Block_A
#define AUDIO_IN_SAI_CLK_ENABLE()                                __SAI1_CLK_ENABLE()
#define AUDIO_IN_SAI_SCK_PIN                                     GPIO_PIN_8
#define AUDIO_IN_SAI_SCK_GPIO_PORT                               GPIOA
#define AUDIO_IN_SAI_SCK_GPIO_CLK_ENABLE()                       __GPIOA_CLK_ENABLE()
#define AUDIO_IN_SAI_SCK_AF                                      GPIO_AF3_SAI1  
#define AUDIO_IN_SAI_SD_PIN                                    GPIO_PIN_10
#define AUDIO_IN_SAI_SD_GPIO_PORT                              GPIOA
#define AUDIO_IN_SAI_SD_GPIO_CLK_ENABLE()                      __GPIOA_CLK_ENABLE();
#define AUDIO_IN_SAI_SD_AF                                     GPIO_AF3_SAI1
#define AUDIO_IN_SAI_SD2_PIN                                    GPIO_PIN_9
#define AUDIO_IN_SAI_SD2_GPIO_PORT                              GPIOA
#define AUDIO_IN_SAI_SD2_GPIO_CLK_ENABLE()                      __GPIOA_CLK_ENABLE();
#define AUDIO_IN_SAI_SD2_AF                                     GPIO_AF3_SAI1
  
  /* I2S DMA definitions */
#define AUDIO_IN_SAI_DMAx_CLK_ENABLE()                          __DMA1_CLK_ENABLE()
#define AUDIO_IN_SAI_DMAx_STREAM                                DMA1_Stream3
#define AUDIO_IN_SAI_DMAx_CHANNEL                               DMA_CHANNEL_0
#define AUDIO_IN_SAI_DMAx_IRQ                                   DMA1_Stream3_IRQn
#define AUDIO_IN_SAI_DMAx_PERIPH_DATA_SIZE                      DMA_PDATAALIGN_HALFWORD
#define AUDIO_IN_SAI_DMAx_MEM_DATA_SIZE                         DMA_MDATAALIGN_HALFWORD  
#define AUDIO_IN_SAI_IRQHandler                                 DMA1_Stream3_IRQHandler

#else

  /* I2S Configuration defines */
#define AUDIO_IN_I2S_INSTANCE                                    SPI2
#define AUDIO_IN_I2S_CLK_ENABLE()                                __SPI2_CLK_ENABLE()
#define AUDIO_IN_I2S_SCK_PIN                                     GPIO_PIN_13
#define AUDIO_IN_I2S_SCK_GPIO_PORT                               GPIOB
#define AUDIO_IN_I2S_SCK_GPIO_CLK_ENABLE()                       __GPIOB_CLK_ENABLE()
#define AUDIO_IN_I2S_SCK_AF                                      GPIO_AF5_SPI2  
#define AUDIO_IN_I2S_MOSI_PIN                                    GPIO_PIN_15
#define AUDIO_IN_I2S_MOSI_GPIO_PORT                              GPIOB
#define AUDIO_IN_I2S_MOSI_GPIO_CLK_ENABLE()                      __GPIOB_CLK_ENABLE();
#define AUDIO_IN_I2S_MOSI_AF                                     GPIO_AF5_SPI2
  
  /* I2S DMA definitions */
#define AUDIO_IN_I2S_DMAx_CLK_ENABLE()                          __DMA1_CLK_ENABLE()
#define AUDIO_IN_I2S_DMAx_STREAM                                DMA1_Stream3
#define AUDIO_IN_I2S_DMAx_CHANNEL                               DMA_CHANNEL_0
#define AUDIO_IN_I2S_DMAx_IRQ                                   DMA1_Stream3_IRQn
#define AUDIO_IN_I2S_DMAx_PERIPH_DATA_SIZE                      DMA_PDATAALIGN_HALFWORD
#define AUDIO_IN_I2S_DMAx_MEM_DATA_SIZE                         DMA_MDATAALIGN_HALFWORD  
#define AUDIO_IN_I2S_IRQHandler                                 DMA1_Stream3_IRQHandler

#endif


/* DFSDM Configuration defines */
#define AUDIO_DFSDMx_MIC1_CHANNEL                    DFSDM_Channel2  
#define AUDIO_DFSDMx_MIC2_CHANNEL                    DFSDM_Channel1
#define AUDIO_DFSDMx_MIC3_CHANNEL                    DFSDM_Channel7  
#define AUDIO_DFSDMx_MIC4_CHANNEL                    DFSDM_Channel6

#define AUDIO_DFSDMx_MIC1_CHANNEL_FOR_FILTER         DFSDM_CHANNEL_2  
#define AUDIO_DFSDMx_MIC2_CHANNEL_FOR_FILTER         DFSDM_CHANNEL_1
#define AUDIO_DFSDMx_MIC3_CHANNEL_FOR_FILTER         DFSDM_CHANNEL_7  
#define AUDIO_DFSDMx_MIC4_CHANNEL_FOR_FILTER         DFSDM_CHANNEL_6

#define AUDIO_DFSDMx_MIC1_FILTER                     DFSDM_Filter0
#define AUDIO_DFSDMx_MIC2_FILTER                     DFSDM_Filter1
#define AUDIO_DFSDMx_MIC3_FILTER                     DFSDM_Filter2
#define AUDIO_DFSDMx_MIC4_FILTER                     DFSDM_Filter3
#define AUDIO_DFSDMx_CLK_ENABLE()                    __HAL_RCC_DFSDM_CLK_ENABLE()

/* DATIN for MIC1 */
#define AUDIO_DFSDMx_DATIN_MIC1_PIN                  GPIO_PIN_14
#define AUDIO_DFSDMx_DATIN_MIC1_AF                   GPIO_AF6_DFSDM
#define AUDIO_DFSDMx_DATIN_MIC1_GPIO_PORT            GPIOB
#define AUDIO_DFSDMx_DATIN_MIC1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE() 

/* DATIN for MIC2 */
#define AUDIO_DFSDMx_DATIN_MIC2_PIN                  GPIO_PIN_14
#define AUDIO_DFSDMx_DATIN_MIC2_AF                   GPIO_AF6_DFSDM
#define AUDIO_DFSDMx_DATIN_MIC2_GPIO_PORT            GPIOB
#define AUDIO_DFSDMx_DATIN_MIC2_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE() 

/* DATIN for MIC3 */
#define AUDIO_DFSDMx_DATIN_MIC3_PIN                  GPIO_PIN_10
#define AUDIO_DFSDMx_DATIN_MIC3_AF                   GPIO_AF6_DFSDM
#define AUDIO_DFSDMx_DATIN_MIC3_GPIO_PORT            GPIOB
#define AUDIO_DFSDMx_DATIN_MIC3_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE() 

/* DATIN for MIC4 */
#define AUDIO_DFSDMx_DATIN_MIC4_PIN                  GPIO_PIN_10
#define AUDIO_DFSDMx_DATIN_MIC4_AF                   GPIO_AF6_DFSDM
#define AUDIO_DFSDMx_DATIN_MIC4_GPIO_PORT            GPIOB
#define AUDIO_DFSDMx_DATIN_MIC4_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()


/* CKOUT for all mics (GPIOC_PIN_2)*/                                                           
#define AUDIO_DFSDMx_CKOUT_PIN                       GPIO_PIN_2
#define AUDIO_DFSDMx_CKOUT_AF                        GPIO_AF6_DFSDM
#define AUDIO_DFSDMx_CKOUT_GPIO_PORT                 GPIOC
#define AUDIO_DFSDMx_CKOUT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()


/* DFSDM DMA MIC1 and MIC2 channels definitions */
#define AUDIO_DFSDMx_DMAx_MIC1_STREAM                DMA1_Channel4
#define AUDIO_DFSDMx_DMAx_MIC1_IRQ                   DMA1_Channel4_IRQn
#define AUDIO_DFSDM_DMAx_MIC1_IRQHandler             DMA1_Channel4_IRQHandler

#define AUDIO_DFSDMx_DMAx_MIC2_STREAM                DMA1_Channel5
#define AUDIO_DFSDMx_DMAx_MIC2_IRQ                   DMA1_Channel5_IRQn
#define AUDIO_DFSDM_DMAx_MIC2_IRQHandler             DMA1_Channel5_IRQHandler

#define AUDIO_DFSDMx_DMAx_MIC3_STREAM                DMA1_Channel6
#define AUDIO_DFSDMx_DMAx_MIC3_IRQ                   DMA1_Channel6_IRQn
#define AUDIO_DFSDM_DMAx_MIC3_IRQHandler             DMA1_Channel6_IRQHandler

#define AUDIO_DFSDMx_DMAx_MIC4_STREAM                DMA1_Channel7
#define AUDIO_DFSDMx_DMAx_MIC4_IRQ                   DMA1_Channel7_IRQn
#define AUDIO_DFSDM_DMAx_MIC4_IRQHandler             DMA1_Channel7_IRQHandler

#define AUDIO_DFSDMx_DMAx_PERIPH_DATA_SIZE           DMA_PDATAALIGN_WORD
#define AUDIO_DFSDMx_DMAx_MEM_DATA_SIZE              DMA_MDATAALIGN_WORD
#define AUDIO_DFSDMx_DMAx_CLK_ENABLE()               __HAL_RCC_DMA1_CLK_ENABLE()                                                     

#ifdef USE_SPI3
    /* SPI Configuration defines */  
  
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
  
#else
  /* SPI Configuration defines */  
#define AUDIO_IN_SPI_INSTANCE                                            SPI1
#define AUDIO_IN_SPI_CLK_ENABLE()                               __SPI1_CLK_ENABLE()
#define AUDIO_IN_SPI_SCK_GPIO_CLK_ENABLE()                      __GPIOA_CLK_ENABLE()
#define AUDIO_IN_SPI_MISO_GPIO_CLK_ENABLE()                     __GPIOA_CLK_ENABLE() 
#define AUDIO_IN_SPI_MOSI_GPIO_CLK_ENABLE()                     __GPIOA_CLK_ENABLE()   
#define AUDIO_IN_SPI_FORCE_RESET()                              __SPI1_FORCE_RESET()
#define AUDIO_IN_SPI_RELEASE_RESET()                            __SPI1_RELEASE_RESET()
#define AUDIO_IN_SPI_SCK_PIN                                    GPIO_PIN_5
#define AUDIO_IN_SPI_SCK_GPIO_PORT                              GPIOA
#define AUDIO_IN_SPI_SCK_AF                                     GPIO_AF5_SPI1
#define AUDIO_IN_SPI_MOSI_PIN                                   GPIO_PIN_7
#define AUDIO_IN_SPI_MOSI_GPIO_PORT                             GPIOA
#define AUDIO_IN_SPI_MOSI_AF                                    GPIO_AF5_SPI1
  
  /* SPI DMA definitions */
#define AUDIO_IN_SPI_DMAx_CLK_ENABLE()                          __DMA2_CLK_ENABLE()
#define AUDIO_IN_SPI_RX_DMA_CHANNEL                             DMA_CHANNEL_3
#define AUDIO_IN_SPI_RX_DMA_STREAM                              DMA2_Stream2 
#define AUDIO_IN_SPI_DMA_RX_IRQn                                DMA2_Stream2_IRQn
#define AUDIO_IN_SPI_DMA_RX_IRQHandler                          DMA2_Stream2_IRQHandler
  
#endif

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
#ifndef AUDIO_IN_CHANNELS
#define AUDIO_IN_CHANNELS 1
#endif

#ifndef AUDIO_IN_SAMPLING_FREQUENCY
#define AUDIO_IN_SAMPLING_FREQUENCY 16000
#endif

#ifndef AUDIO_IN_BIT_DEPTH 
#define AUDIO_IN_BIT_DEPTH AUDIO_RESOLUTION_16b
#endif

#ifndef AUDIO_VOLUME_INPUT
#define AUDIO_VOLUME_INPUT              64U
#endif

#ifndef CCA02M2_AUDIO_INSTANCE
#define CCA02M2_AUDIO_INSTANCE 0U
#endif

#ifndef CCA02M2_AUDIO_IN_IT_PRIORITY
#define CCA02M2_AUDIO_IN_IT_PRIORITY    6U
#endif

#define AUDIO_IN_DIGITAL_MIC1      0x10U
#define AUDIO_IN_DIGITAL_MIC2      0x20U
#define AUDIO_IN_DIGITAL_MIC3      0x40U
#define AUDIO_IN_DIGITAL_MIC4      0x80U
#define AUDIO_IN_DIGITAL_MIC_LAST  AUDIO_IN_DIGITAL_MIC4
#define AUDIO_IN_DIGITAL_MIC       (AUDIO_IN_DIGITAL_MIC1 | AUDIO_IN_DIGITAL_MIC2 | AUDIO_IN_DIGITAL_MIC3 | AUDIO_IN_DIGITAL_MIC4)
#define DFSDM_MIC_NUMBER           AUDIO_IN_CHANNELS

#define CHANNEL_DEMUX_MASK                    	0x55U

#ifndef MAX_MIC_FREQ                 	  
#define MAX_MIC_FREQ                 	  3072  /*KHz*/
#endif

#ifndef MAX_AUDIO_IN_CHANNEL_NBR_PER_IF   
#define MAX_AUDIO_IN_CHANNEL_NBR_PER_IF   2
#endif

#ifndef MAX_AUDIO_IN_CHANNEL_NBR_TOTAL    
#define MAX_AUDIO_IN_CHANNEL_NBR_TOTAL    4 /* For WB, this must be minimum equal to 2 */
#endif

#ifndef ENABLE_HIGH_PERFORMANCE_MODE
#define ENABLE_HIGH_PERFORMANCE_MODE 0U
#endif

#if (ENABLE_HIGH_PERFORMANCE_MODE == 1U)
#define PDM_FREQ_16K 1536
#endif

#ifndef PDM_FREQ_16K
#define PDM_FREQ_16K 1280
#endif

/*Number of millisecond of audio at each DMA interrupt*/
#ifndef N_MS_PER_INTERRUPT
#define N_MS_PER_INTERRUPT               (1U)
#endif

/* Default Audio IN internal buffer size */   
#define DEFAULT_AUDIO_IN_BUFFER_SIZE        (uint32_t)((AUDIO_IN_SAMPLING_FREQUENCY/1000)*2)*N_MS_PER_INTERRUPT    

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
   Instance 0 is SAI-I2S / SPI path
   Instance 1 is DFSDM path
   Instance 2 is PDM path
 */
#define AUDIO_IN_INSTANCES_NBR             3U
/**
  * @}
  */
   
/** @defgroup CCA02M2_AUDIO_Exported_Macros CCA02M2_AUDIO_ Exported Macros
  * @{
  */
#ifndef POS_VAL
#define POS_VAL(VAL)                  (POSITION_VAL(VAL) - 4U)
#endif
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 239) / 100)))
    
/**
  * @}
  */ 
/** @addtogroup CCA02M2_AUDIO_Exported_Variables
  * @{
  */
/* Recording context */
extern AUDIO_IN_Ctx_t                         AudioInCtx[];
/**
  * @}
  */

/** @defgroup CCA02M2_AUDIO_IN_Exported_Functions CCA02M2_AUDIO_IN Exported Functions
  * @{
  */
int32_t CCA02M2_AUDIO_IN_Init(uint32_t Instance, CCA02M2_AUDIO_Init_t* AudioInit);    
int32_t CCA02M2_AUDIO_IN_DeInit(uint32_t Instance);
#if ((USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U) || (USE_HAL_SAI_REGISTER_CALLBACKS == 1U))
int32_t CCA02M2_AUDIO_IN_RegisterDefaultMspCallbacks (uint32_t Instance);
int32_t CCA02M2_AUDIO_IN_RegisterMspCallbacks (uint32_t Instance, CCA02M2_AUDIO_IN_Cb_t *CallBacks);
#endif /* ((USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U) || (USE_HAL_SAI_REGISTER_CALLBACKS == 1U)) */
int32_t CCA02M2_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes);
int32_t CCA02M2_AUDIO_IN_Stop(uint32_t Instance);
int32_t CCA02M2_AUDIO_IN_Pause(uint32_t Instance);
int32_t CCA02M2_AUDIO_IN_Resume(uint32_t Instance);

int32_t CCA02M2_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes);
int32_t CCA02M2_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device);
int32_t CCA02M2_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device);
int32_t CCA02M2_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device);

int32_t CCA02M2_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device);
int32_t CCA02M2_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t CCA02M2_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t CCA02M2_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);                 
int32_t CCA02M2_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t CCA02M2_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);                
int32_t CCA02M2_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t CCA02M2_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t CCA02M2_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t CCA02M2_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t CCA02M2_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State);

/* Specific PDM recodr APIs */
int32_t CCA02M2_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut);
int32_t CCA02M2_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf);
int32_t CCA02M2_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes);

void CCA02M2_AUDIO_IN_IRQHandler(uint32_t Instance, uint32_t Device);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
   It is called into this driver when the current buffer is filled to prepare the next
   buffer pointer and its size. */
void CCA02M2_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);
void CCA02M2_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void CCA02M2_AUDIO_IN_Error_CallBack(uint32_t Instance);

/**
  * @}
  */ 
  
/** @defgroup CCA02M2_AUDIO_IN_Private_Functions CCA02M2_AUDIO_IN Private Functions
  * @{
  */ 
  
#ifdef USE_STM32L4XX_NUCLEO

/* These function can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
HAL_StatusTypeDef MX_DFSDM1_ClockConfig(DFSDM_Channel_HandleTypeDef *hDfsdmChannel, uint32_t SampleRate);
HAL_StatusTypeDef MX_DFSDM1_Init(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel, MX_DFSDM_Config *MXConfig);

#else

#ifdef USE_STM32WBXX_NUCLEO
HAL_StatusTypeDef MX_SAI_ClockConfig(SAI_HandleTypeDef *hSai, uint32_t PDM_rate);
#else
HAL_StatusTypeDef MX_I2S_IN_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t PDM_rate);
HAL_StatusTypeDef MX_I2S_IN_Init(I2S_HandleTypeDef* hi2s, MX_I2S_IN_Config *MXConfig);
HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef* hspi, MX_SPI_Config *MXConfig);
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

#ifdef __cplusplus
}
#endif

#endif /* CCA02M2_AUDIO_H */
