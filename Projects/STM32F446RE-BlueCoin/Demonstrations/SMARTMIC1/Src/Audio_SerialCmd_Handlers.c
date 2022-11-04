/**
  ******************************************************************************
  * @file    Audio_SerialCmd_Handlers.c
  * @author  SRA
  * 
  * 
  * @brief   This file provides set of firmware functions to handle serial
  *          commands relevant to audio status.
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

/* Includes ------------------------------------------------------------------*/
#include "Audio_SerialCmd_Handlers.h"

/** @defgroup SMARTMIC1
  * @{
  */

/** @defgroup SMARTMIC1_COMMUNICATION
  * @{
  */

/** @defgroup SMARTMIC1_COMMUNICATION_Private_Variables
  * @{
  */
static AudioStatus_t InternalAudioStatus;
static uint8_t AudioStatusLastChanged;
/**
  * @}
  */

/** @defgroup SMARTMIC1_COMMUNICATION_Exported_Functions
  * @{
  */

/**
  * @brief  Handle the Set Status message, copying the new status to the
  *         InternalStatus  instance
  * @param  Msg pointer for the msg
  * @retval 0
  */
int32_t Handle_CMD_AUDIO_IN_SetStatus(TMsg *Msg)
{
  uint8_t StatusDomain = Msg->Data[3];
  AudioStatusLastChanged = StatusDomain;
  uint8_t *pdata = &(Msg->Data[4]);

  if (StatusDomain & DOMAIN_GENERAL)
  {
    memcpy(&(InternalAudioStatus.GeneralStatus), pdata, sizeof(GeneralParam_t));
    pdata += sizeof(GeneralParam_t);
  }

  if (StatusDomain & DOMAIN_SLOC)
  {
    memcpy(&(InternalAudioStatus.SLocStatus), pdata, sizeof(SLocParam_t));
    pdata += sizeof(SLocParam_t);
  }

  if (StatusDomain & DOMAIN_BEAMFORMING)
  {
    memcpy(&(InternalAudioStatus.BeamStatus), pdata, sizeof(BeamParam_t));
    pdata += sizeof(BeamParam_t);
  }

  if (StatusDomain & DOMAIN_AEC)
  {
    memcpy(&(InternalAudioStatus.AECStatus), pdata, sizeof(AECParam_t));
    pdata += sizeof(AECParam_t);
  }

  if (StatusDomain & DOMAIN_DBNOISE)
  {
    memcpy(&(InternalAudioStatus.dBStatus), pdata, sizeof(dBParam_t));
    pdata += sizeof(dBParam_t);
  }

  if (StatusDomain & DOMAIN_ASR)
  {
    memcpy(&(InternalAudioStatus.ASRStatus), pdata, sizeof(ASRParam_t));
    pdata += sizeof(ASRParam_t);
  }

  if (StatusDomain & DOMAIN_OUTPUT)
  {
    memcpy(&(InternalAudioStatus.OutputStatus), pdata, sizeof(OutputParam_t));
    pdata += sizeof(OutputParam_t);
  }

  return 0;
}

/**
  * @brief  Handle the Get Status message, copying the InternalStatus instance
  *         status to the message to be sent
  * @param  Msg pointer for the msg
  * @retval 0
  */
int32_t Handle_CMD_AUDIO_IN_GetStatus(TMsg *Msg)
{

  Msg->Len = 4;
  uint8_t StatusDomain = Msg->Data[3];
  uint8_t *pdata = &(Msg->Data[4]);

  if (StatusDomain & DOMAIN_GENERAL)
  {
    memcpy(pdata, &(InternalAudioStatus.GeneralStatus), sizeof(GeneralParam_t));
    pdata += sizeof(GeneralParam_t);
    Msg->Len += sizeof(GeneralParam_t);
  }

  if (StatusDomain & DOMAIN_SLOC)
  {
    memcpy(pdata, &(InternalAudioStatus.SLocStatus), sizeof(SLocParam_t));
    pdata += sizeof(SLocParam_t);
    Msg->Len += sizeof(SLocParam_t);
  }

  if (StatusDomain & DOMAIN_BEAMFORMING)
  {
    memcpy(pdata, &(InternalAudioStatus.BeamStatus), sizeof(BeamParam_t));
    pdata += sizeof(BeamParam_t);
    Msg->Len += sizeof(BeamParam_t);
  }

  if (StatusDomain & DOMAIN_AEC)
  {
    memcpy(pdata, &(InternalAudioStatus.AECStatus), sizeof(AECParam_t));
    pdata += sizeof(AECParam_t);
    Msg->Len += sizeof(AECParam_t);
  }
  if (StatusDomain & DOMAIN_DBNOISE)
  {
    memcpy(pdata, &(InternalAudioStatus.dBStatus), sizeof(dBParam_t));
    pdata += sizeof(dBParam_t);
    Msg->Len += sizeof(dBParam_t);
  }

  if (StatusDomain & DOMAIN_ASR)
  {
    memcpy(pdata, &(InternalAudioStatus.ASRStatus), sizeof(ASRParam_t));
    pdata += sizeof(ASRParam_t);
    Msg->Len += sizeof(ASRParam_t);
  }

  if (StatusDomain & DOMAIN_OUTPUT)
  {
    memcpy(pdata, &(InternalAudioStatus.OutputStatus), sizeof(OutputParam_t));
    pdata += sizeof(OutputParam_t);
    Msg->Len += sizeof(OutputParam_t);
  }
  return 0;
}

/**
  * @brief  Used to retrieve the internal status
  * @param  None
  * @retval pointer to the internal status
  */
AudioStatus_t *GetAudioStatusInternal(void)
{
  return &InternalAudioStatus;
}

/**
  * @brief  Returns a byte containing the last chages occurred to the Internal
  *         Status after a set status request
  * @param  None
  * @retval byte describing the changes happened in the status
  */
uint8_t GetAudioStatusLastChanged(void)
{
  return AudioStatusLastChanged;
}

/**
  * @brief  Resets the byte containing the last chages occurred to the Internal
  *         Status
  * @param  none
  * @retval none
  */
void ResetAudioStatusLastChanged(void)
{
  AudioStatusLastChanged = 0;
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
