/**
  ******************************************************************************
  * @file    usbd_audio_cdc_if_template.h
  * @author  SRA
  * @brief   Header for usbd_audio_cdc_if_template.c file.
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
#ifndef __USBD_AUDIO_CDC_IF_TEMPLATE_H
#define __USBD_AUDIO_CDC_IF_TEMPLATE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_cdc.h"
  
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Send_Audio_to_USB(int16_t * audioData, uint16_t PCMSamples);

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_AUDIO_CDC_IF_TEMPLATE_H */

