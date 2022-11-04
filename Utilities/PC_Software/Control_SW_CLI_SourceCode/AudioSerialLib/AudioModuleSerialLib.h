/**
  ******************************************************************************
  * @file    
  * @author  SRA
  * @brief 
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
  
#ifndef AUDIO_MODULE_SERIAL_LIB_H
#define AUDIO_MODULE_SERIAL_LIB_H

#include <stdint.h>
#include <ASTSerialLib.h>
#include <SerialComm.h>
#include <stddef.h>


/** @addtogroup SERIALCMD_HANDLERS
* @{
*/

/** @addtogroup AUDIO_STATUS
* @{
*/

/** @defgroup Audio Status Domains definition
* @brief Audio Status Domains definition
* @{
*/
#define DOMAIN_GENERAL   			(0x0001)
#define DOMAIN_BEAMFORMING 			(0x0002)
#define DOMAIN_SLOC                             (0x0004)
#define DOMAIN_DBNOISE                          (0x0008)
#define DOMAIN_AEC                              (0x0010)
#define DOMAIN_ASR                              (0x0020)
#define DOMAIN_OUTPUT                           (0x0040)
/**
* @}
*/

/** @defgroup Audio Status Channels definition
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

/** @defgroup Audio Status Beamforming directions definition
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

/** @defgroup Audio Status Algorithms definition
* @brief Audio Status Algorithms definition
* @{
*/
#define ALGO_ACTIVATION_BF                         (0x0001)
#define ALGO_ACTIVATION_SL                         (0x0002)
#define ALGO_ACTIVATION_EC                         (0x0004)
#define ALGO_ACTIVATION_DB                         (0x0008)
#define ALGO_ACTIVATION_ASR                        (0x0010)
/**
* @}
*/

/** @defgroup Audio Status output definition
* @brief Audio Status output definition
* @{
*/
#define AUDIOOUT_STATUS_STOP                         (0x00)
#define AUDIOOUT_STATUS_SONG                         (0x01)
#define AUDIOOUT_STATUS_MICS                         (0x02)
/**
* @}
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
TGeneralParam;


/**
* @brief  Source Localization Status Structure definition
*/
typedef struct
{
    int16_t Angle;                      /*!<Computed angle*/
    uint8_t Algorithm;                  /*!<Running algorithm*/
    uint8_t Resolution;                 /*!<Current resolution*/
    int16_t Threshold;                   /*!<Current threshold*/
    uint32_t Reserved[4];               /*!<For future use*/
}
TSLocParam;


/**
* @brief Beamforming Status Structure definition
*/
typedef struct
{
    uint8_t Type;                       /*!<Current Beamforming Typre*/
    uint8_t Direction;                  /*!<Current Beamforming Direction*/
    float Gain;                         /*!<Current Beamforming Gain*/
    uint32_t Reserved[4];               /*!<For future use*/
}
TBeamParam;

/**
* @brief AEC Status Structure definition
*/
typedef struct
{
    uint16_t TailLength;                /*!<Current Tail Length*/
    uint16_t AGCValue;                  /*!<Current AGC Value */
    uint16_t Denoiser;                  /*!<Current Denoiser State*/
    uint32_t Reserved[4];               /*!<For future use*/
}
TAECParam;

/**
* @brief  dBSPL estimation Status Structure definition
*/
typedef struct
{
    int16_t dBValue;                    /*!<Computed angle*/
    int16_t offset;                     /*!<Offset*/
    uint32_t Reserved[4];               /*!<For future use*/
}
TdBParam;

/**
* @brief  ASR Status Structure definition
*/
typedef struct
{
    int16_t RecognizedWord;             /*!<Recognized word*/
    uint32_t Reserved[4];               /*!<For future use*/
}
TASRParam;

/**
* @brief  Output Status Structure definition
*/
typedef struct
{
    uint8_t Status;                     /*!<Status*/
    uint8_t Volume;                     /*!<Volume*/
}
TOutputParam;

/**
* @brief  Overall Status Structure definition
*/
typedef struct
{
    TGeneralParam GeneralStatus;        /*!<Genral Status*/
    TSLocParam SLocStatus;              /*!<SLoc Status*/
    TBeamParam BeamStatus;              /*!<Beam Status*/
    TAECParam AECStatus;                /*!<AEC Status*/
    TdBParam dBStatus;                  /*!<dbSPL Status*/
    TASRParam ASRStatus;                /*!<ASR Status*/
    TOutputParam OutputStatus;          /*!<Out Status*/
}
TAudioStatus;



class AudioModuleSerialLib : public ASTSerialLib
{
public:

    AudioModuleSerialLib();
    ~AudioModuleSerialLib();


    /**
* This function is used in order to  set the device status.<br>
* @param[in] DestDevAddr: AudioModule address, unsigned char: {0-255}.
* @param[in] StatusDomain: mask defining the status domains to be sent to the device.
* @param[in] Status: status to be sent.
* @return Error code.
*/
    int AudioModuleCmd_SetStatus(unsigned char DestDevAddr, uint8_t StatusDomain, TAudioStatus *Status);

    /**
* This function is used in order to get the device status.<br>
* @param[in] DestDevAddr: AudioModule address, unsigned char: {0-255}.
* @param[in] StatusDomain: mask defining the status domains to be recieved from the device.
* @param[in] Status: status that will be filled with the info from the device.
* @return Error code.
*/
    int AudioModuleCmd_GetStatus(unsigned char DestDevAddr, uint8_t StatusDomain, TAudioStatus *Status);

};


/**
* @}
*/

/**
* @}
*/

#endif // AUDIO_MODULE_SERIAL_LIB_H

