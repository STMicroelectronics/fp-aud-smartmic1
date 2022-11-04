/**
  ******************************************************************************
  * @file    cca01m1_conf.h
  * @author  SRA
  * 
  * 
  * @brief   This file contains definitions for the SOUNDTER1 applications
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
#ifndef CCA01M1_CONF_H__
#define CCA01M1_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "nucleo_f446re_bus.h"
#include "nucleo_f446re_errno.h"


#define AUDIO_OUT_CHANNELS    2
#define AUDIO_OUT_SAMPLING_FREQUENCY  32000

#define AUDIO_OUT_BUFFER_SIZE           (AUDIO_OUT_SAMPLING_FREQUENCY/1000 * 2 * 8)
#define AUDIO_VOLUME_OUT          30U
#define CCA01M1_AUDIO_OUT_INSTANCE  1U
#define CCA01M1_AUDIO_OUT_IT_PRIORITY   6U

#ifdef __cplusplus
}
#endif

#endif /* CCA01M1_CONF_H__*/



