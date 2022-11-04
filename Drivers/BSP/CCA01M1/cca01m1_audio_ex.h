/**
  ******************************************************************************
  * @file    cca01m1_audio_ex.h
  * @author  SRA
  * @brief   This file contains definitions for cca01m1_audio_ex.c
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
#ifndef CCA01M1_AUDIO_EX_H
#define CCA01M1_AUDIO_EX_H

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


/** @defgroup  X_NUCLEO_CCA01M1_AUDIO_Exported_Functions Exported Functions 
* @{
*/
int32_t CCA01M1_AUDIO_OUT_SetEq(uint32_t Instance, uint8_t ramBlock, uint8_t filterNumber, uint32_t * filterValues);
int32_t CCA01M1_AUDIO_OUT_SetTone(uint32_t Instance, uint8_t toneGain);
int32_t CCA01M1_AUDIO_OUT_SetDSPOption(uint32_t Instance, uint8_t option, uint8_t state);

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

#endif /* CCA01M1_AUDIO_EX_H */

