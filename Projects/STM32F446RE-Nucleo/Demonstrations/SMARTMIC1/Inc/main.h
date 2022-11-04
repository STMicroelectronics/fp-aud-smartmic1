/**
******************************************************************************
* @file    main.h 
* @author  SRA
* 
* 
* @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "usbd_desc.h"

#include "cca01m1_audio.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_nucleo_USART.h"

#ifdef USE_COMPOSITE_VCP_AUDIO
#include "usbd_audio_cdc_interface.h"
#include "USB_protocol_interface.h"
#else
#include "usbd_audio_in.h"
#include "usbd_audio_if.h"
#include "UART_protocol_interface.h"
#endif

#include "audio_application.h"
#include "STCmdP_interpreter.h"

/* Exported Defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#endif /* __MAIN_H */


