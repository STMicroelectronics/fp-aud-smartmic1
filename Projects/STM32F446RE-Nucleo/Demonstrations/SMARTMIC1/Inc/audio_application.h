/**
  ******************************************************************************
  * @file    audio_application.h
  * @author  SRA
  * 
  * 
  * @brief   Header for audio_application.c module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *                             
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIO_APPLICATION_H
#define __AUDIO_APPLICATION_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "acoustic_bf.h"
#include "acoustic_ec.h"
#include "acoustic_sl.h"
#include "acoustic_db.h"
#include "math.h"
#include "fragment1.h"
#include "fragment2.h"
#include "fragment3.h"
#include "audio_song.h"
#include "Audio_SerialCmd_Handlers.h"
#include "arm_math.h"

#define AUDIO_OUT_DEVICE STA350BW_1
#define AUDIO_OUT_INTERRUPT AUDIO_OUT2_I2S_DMAx_IRQ
#define COUPONS

#if defined (COUPONS)
#define TOP_LEFT_MIC 3
#define TOP_RIGHT_MIC 0
#define BOTTOM_LEFT_MIC 2
#define BOTTOM_RIGHT_MIC 1
#define SIDE_X 147
#define SIDE_Y 147
#define DIAGONAL 212

#elif defined (u4_ARRAY)
#define TOP_RIGHT_MIC 0
#define TOP_LEFT_MIC 1
#define BOTTOM_RIGHT_MIC 2
#define BOTTOM_LEFT_MIC 3
#define SIDE_X 40
#define SIDE_Y 40
#define DIAGONAL 57

#endif

#define LED0_MASK ((uint8_t)0x01)
#define LED45_MASK ((uint8_t)0x02)
#define LED90_MASK ((uint8_t)0x04)
#define LED135_MASK ((uint8_t)0x08)
#define LED180_MASK ((uint8_t)0x10)
#define LED225_MASK ((uint8_t)0x20)
#define LED270_MASK ((uint8_t)0x40)
#define LED315_MASK ((uint8_t)0x80)



#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
#define SONG_NUMBER 2



/** @addtogroup SMARTMIC1
  * @{
  */

/** @defgroup SMARTMIC1_AUDIO
  * @{
  */

/** @defgroup SMARTMIC1_AUDIO_Private_Types
  * @{
  */

/**
  * @brief  Local Parameters Data Structure
  */
typedef struct
{
  uint8_t BF1stMicIndex;                /*!<Index of the 1st mic for BF*/
  uint8_t BF2ndtMicIndex;               /*!<Index of the 2nd mic for BF*/
  uint8_t AudioOutStatus;               /*!<Current out status (Mics or Song)*/
  uint16_t AudioOutWrPtr;               /*!<write position in the out buffer*/
  uint16_t AudioOutRdPtr;               /*!<read position in the out buffer*/
  uint16_t RunningAlgos;                /*!<Algorithms currently running*/
  uint16_t AEC_OUT_started;
  uint8_t LOCK;                        /*!<lock to avoid race conditions*/
}
InternalParams_t;
/**
  * @}
  */

/** @defgroup SMARTMIC1_AUDIO_Exported_Defines
  * @{
  */


/**
  * @}
  */


/** @defgroup SMARTMIC1_AUDIO_Exported_Functions
  * @{
  */
void Error_Handler(void);
void AudioProcess(void);
void SW_IRQ_Tasks_Init(void);
void SW_IRQ_Tasks_Enable(void);
void SW_IRQ_Tasks_Disable(void);
void SW_Task1_Callback(void);
void SW_Task2_Callback(void);
void SW_Task1_Start(void);
void SW_Task2_Start(void);
void SW_Task3_Callback(void);
void SW_Task3_Start(void);
void SW_Task4_Start(void);
void SW_Task4_Callback(void);

uint32_t BF_Init(void);
uint32_t SL_Init(void);
uint32_t EC_Init(void);
uint32_t DB_Init(void);
void BF_DeInit(void);
void SL_DeInit(void);
void EC_DeInit(void);

void AudioStatus_Init(void);
uint32_t InitializeApplication(void);
uint8_t AudioDevices_Init(void);
uint8_t StartMicAcquisition(void);

void Songs_Init(void);
void PeriodicAudioStatusManager(void);

void LedsOff(uint8_t LedMask);
void LedsOn(uint8_t LedMask);
void LedsToggle(uint8_t LedMask);

void BeamDirectionSetup(uint8_t direction);

uint8_t BSP_AUDIO_IN_PDMToPCM_DB_Noise_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut);
uint8_t BSP_AUDIO_IN_PDMToPCM_DB_Noise(uint16_t *PDMBuf, uint16_t *PCMBuf);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */



#endif /* __AUDIO_APPLICATION_H */


