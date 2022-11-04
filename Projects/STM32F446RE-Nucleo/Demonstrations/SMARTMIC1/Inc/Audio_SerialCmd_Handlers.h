/**
  ******************************************************************************
  * @file    Audio_SerialCmd_Handlers.h
  * @author  SRA
  * 
  * 
  * @brief   This file contains definitions for AudioIn_SerialCmd_Handlers.c file.
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
#ifndef __AUDIO_SERIALCMD_HANDLERS
#define __AUDIO_SERIALCMD_HANDLERS

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
#include "STCmdP.h"


/** @defgroup SMARTMIC1
  * @{
  */

/** @defgroup SMARTMIC1_COMMUNICATION
  * @{
  */

/** @defgroup SMARTMIC1_COMMUNICATION_Exported_Constants
  * @{
  */

/**
  * @brief Audio Status Domains definition
  * @{
  */
#define DOMAIN_GENERAL        (0x0001)
#define DOMAIN_BEAMFORMING      (0x0002)
#define DOMAIN_SLOC                             (0x0004)
#define DOMAIN_DBNOISE                          (0x0008)
#define DOMAIN_AEC                              (0x0010)
#define DOMAIN_ASR                              (0x0020)
#define DOMAIN_OUTPUT                           (0x0040)
/**
  * @}
  */

/**
  * @brief Audio Status Channels definition
  * @{
  */
#define CH_MIC1                                    (0x00000001)
#define CH_MIC2                                    (0x00000002)
#define CH_MIC3                                    (0x00000004)
#define CH_MIC4                                    (0x00000008)
#define CH_MIC5                                    (0x00000010)
#define CH_MIC6                                    (0x00000020)
#define CH_MIC7                                    (0x00000040)
#define CH_MIC8                                    (0x00000080)
#define CH_MIC9                                    (0x00000100)
#define CH_MIC10                                   (0x00000200)
#define CH_MIC11                                   (0x00000400)
#define CH_MIC12                                   (0x00000800)
#define CH_MIC13                                   (0x00001000)
#define CH_MIC14                                   (0x00002000)
#define CH_MIC15                                   (0x00004000)
#define CH_MIC16                                   (0x00008000)
#define CH_BF                                      (0x00010000)
#define CH_AEC                                     (0x00020000)
#define CH_RES1                                    (0x00040000)
#define CH_RES2                                    (0x00080000)
#define CH_RES3                                    (0x00100000)
#define CH_RES4                                    (0x00200000)
#define CH_RES6                                    (0x00400000)
#define CH_RES7                                    (0x00800000)
/**
  * @}
  */

/**
  * @brief Audio Status Beamforming directions definition
  * @{
  */
#define BF_DIR1                                    (0x01)
#define BF_DIR2                                    (0x02)
#define BF_DIR3                                    (0x04)
#define BF_DIR4                                    (0x08)
#define BF_DIR5                                    (0x10)
#define BF_DIR6                                    (0x20)
#define BF_DIR7                                    (0x40)
#define BF_DIR8                                    (0x80)
/**
  * @}
  */

/**
  * @brief Audio Status Algorithms definition
  * @{
  */
#define ALGO_ACTIVATION_BF                         (0x0001)
#define ALGO_ACTIVATION_SL                         (0x0002)
#define ALGO_ACTIVATION_EC                         (0x0004)
#define ALGO_ACTIVATION_DB                         (0x0008)
#define ALGO_ACTIVATION_ASR                        (0x0010)
#define ALGO_ACTIVATION_TOF                        (0x0020)
/**
  * @}
  */

/**
  * @brief Audio Status output definition
  * @{
  */
#define AUDIOOUT_STATUS_STOP                         (0x01)
#define AUDIOOUT_STATUS_PAUSE                        (0x02)
#define AUDIOOUT_STATUS_SONG                         (0x04)
#define AUDIOOUT_STATUS_MICS                         (0x08)
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup SMARTMIC1_COMMUNICATION_Exported_Types
  * @{
  */

/**
  * @brief  General Status Structure definition
  */
typedef struct
{
  uint16_t AvailableModules;          /*!<Specifies the algorithms available
  in the application */
  uint16_t AlgorithmActivation;       /*!<Specifies the currently running
  algorithm */
  uint8_t Volume;                     /*!<Specifies the current IN volume*/
  uint16_t SamplingFreq;              /*!<Specifies the current Fs*/
  uint8_t ChannelNumber;              /*!<Specifies the current ch number*/
  uint32_t ChannelMask;               /*!<Specifies the current active chs*/
  uint32_t Reserved[4];               /*!<For future use*/
}
GeneralParam_t;


/**
  * @brief  Source Localization Status Structure definition
  */
typedef struct
{
  int16_t Angle;                      /*!<Computed angle*/
  uint8_t Algorithm;                  /*!<Running algorithm*/
  uint8_t Resolution;                 /*!<Current resolution*/
  int16_t Threshold;                   /*!<Current threshold*/
  uint32_t Reserved[4];               /*!< For future use*/
}
SLocParam_t;


/**
  * @brief Beamforming Status Structure definition
  */
typedef struct
{
  uint8_t Type;                       /*!<Current Beamforming Typre*/
  uint8_t Direction;                  /*!<Current Beamforming Direction*/
  float Gain;                         /*!<Current Beamforming Gain*/
  uint32_t Reserved[4];               /*!< For future use*/
}
BeamParam_t;

/**
  * @brief AEC Status Structure definition
  */
typedef struct
{
  uint16_t TailLength;                /*!<Current Tail Length*/
  uint16_t AGCValue;                  /*!<Current AGC Value */
  uint16_t Denoiser;                  /*!<Current Denoiser State*/
  uint32_t Reserved[4];               /*!< For future use*/
}
AECParam_t;

/**
  * @brief  dBSPL estimation Status Structure definition
  */
typedef struct
{
  int16_t dBValue;                    /*!<Computed angle*/
  int16_t offset;                     /*!<Offset*/
  uint32_t Reserved[4];               /*!<For future use*/
}
dBParam_t;

/**
  * @brief  ASR Status Structure definition
  */
typedef struct
{
  int16_t RecognizedWord;             /*!<Recognized word*/
  uint32_t Reserved[4];               /*!<For future use*/
}
ASRParam_t;

/**
  * @brief  Output Status Structure definition
  */
typedef struct
{
  uint8_t Status;                     /*!< Status*/
  uint8_t Volume;                     /*!< Volume*/
}
OutputParam_t;

/**
  * @brief  Overall Status Structure definition
  */
typedef struct
{
  GeneralParam_t GeneralStatus;        /*!<Genral Status*/
  SLocParam_t SLocStatus;              /*!<SLoc Status*/
  BeamParam_t BeamStatus;              /*!<Beam Status*/
  AECParam_t AECStatus;                /*!<AEC Status*/
  dBParam_t dBStatus;                  /*!<dbSPL Status*/
  ASRParam_t ASRStatus;                /*!<ASR Status*/
  OutputParam_t OutputStatus;          /*!<Out Status*/
}
AudioStatus_t;
/**
  * @}
  */

/** @defgroup SMARTMIC1_COMMUNICATION_Exported_Function_Prototypes
  * @{
  */
int32_t Handle_CMD_AUDIO_IN_GetStatus(TMsg *Msg);
int32_t Handle_CMD_AUDIO_IN_SetStatus(TMsg *Msg);
AudioStatus_t *GetAudioStatusInternal(void);
uint8_t GetAudioStatusLastChanged(void);
void ResetAudioStatusLastChanged(void);
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

#endif /* __AUDIO_SERIALCMD_HANDLERS */




