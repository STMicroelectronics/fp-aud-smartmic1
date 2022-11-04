/**
******************************************************************************
* @file    Echo_library.h
* @author  SRA
* @brief   This file contains APIs declarations.
******************************************************************************
* @attention
*
* Copyright (C) 2006 Jean-Marc Valin
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
*
* 3. The name of the author may not be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*
* Portions Copyright (c) 2022 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file in
* the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*                        
******************************************************************************   

*//* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ECHO_LIBRARY_H
#define __ECHO_LIBRARY_H

/* Includes ------------------------------------------------------------------*/
#include "acoustic_ec.h"
#include "defines_ec.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
uint32_t libSpeexAEC_GetLibVersion(char *version);
uint32_t libSpeexAEC_Init(AcousticEC_Handler_t *pHandler);
uint32_t libSpeexAEC_getMemorySize(AcousticEC_Handler_t *pHandler);
uint32_t libSpeexAEC_setConfig(AcousticEC_Handler_t *pHandler, AcousticEC_Config_t *pConfig);
uint32_t libSpeexAEC_getConfig(AcousticEC_Handler_t *pHandler, AcousticEC_Config_t *pConfig);
uint32_t libSpeexAEC_Data_Input(void *ptrPrimary, void *ptrReference, void *ptrBufferOut, AcousticEC_Handler_t *pHandler);
uint32_t libSpeexAEC_Process(AcousticEC_Handler_t *pHandler);

#endif  /*__ECHO_LIBRARY_H*/

