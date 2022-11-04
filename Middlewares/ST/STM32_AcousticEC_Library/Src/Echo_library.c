/**
******************************************************************************
* @file    Echo_library.c
* @author  SRA
* @brief   Acoustic Echo Cancellation core library
******************************************************************************
* Copyright (C) 2005-2006 Jean-Marc Valin
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
* - Neither the name of the Xiph.org Foundation nor the names of its contributors
*   may be used to endorse or promote products derived from this software without
*   specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <arm_math.h>
#include "Echo_library.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#define LIB_VERSION 0x00030100UL  /*3.1.0*/

#define SaturaL(N, L) (((N)<(L))?(L):(N))
#define SaturaH(N, H) (((N)>(H))?(H):(N))
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))


#if defined   (__GNUC__)        /* GNU Compiler */
  #define __ALIGN_END    __attribute__ ((aligned (4)))
  #define __ALIGN_BEGIN
#else
  #define __ALIGN_END
  #if defined   (__CC_ARM)      /* ARM Compiler */
    #define __ALIGN_BEGIN    __align(4)
  #elif defined (__ICCARM__)    /* IAR Compiler */
    #define __ALIGN_BEGIN
  #elif defined  (__TASKING__)  /* TASKING Compiler */
    #define __ALIGN_BEGIN    __align(4)
  #endif /* __CC_ARM */
#endif /* __GNUC__ */

/* Private variables ---------------------------------------------------------*/
typedef struct
{

  FilterBank *filterBank;
  SpeexPreprocessState *den;
  drft_lookup *table_den;
  SpeexEchoState st;
  drft_lookup table;
  uint16_t ptr_primary_channels;
  uint16_t ptr_reference_channels;
  uint16_t ptr_output_channels;
  uint16_t samples_count;
  uint16_t samples_count_output;
  int16_t dir1_buf[ECHO_BUFF * 2];
  int16_t dir2_buf[ECHO_BUFF * 2];
  int16_t e_buf[ECHO_BUFF * 2];
  uint16_t buffer_state;
  uint8_t preprocess_initialized;
  uint8_t PREPROCESS;
  uint8_t ECHO;
  uint16_t tail_length;

} InternalEchoType;

/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
uint32_t libSpeexAEC_GetLibVersion(char *version)
{
  return snprintf(version, 35, "ST AcousticEC v%d.%d.%d", (int)((LIB_VERSION >> 16) & 0xFFUL), (int)((LIB_VERSION >> 8) & 0xFFUL), (int)(LIB_VERSION & 0xFFUL));
}

uint32_t libSpeexAEC_Init(AcousticEC_Handler_t *pHandler)
{
  InternalEchoType *echoInstance = (InternalEchoType *)(pHandler->pInternalMemory);
  uint32_t ret = 0;

  memset(pHandler->pInternalMemory, 0, pHandler->internal_memory_size);

  echoInstance->preprocess_initialized = pHandler->preprocess_init;

  if (pHandler->ptr_primary_channels > 0)
  {
    echoInstance->ptr_primary_channels = pHandler->ptr_primary_channels;
  }
  else
  {
    echoInstance->ptr_primary_channels = 1;
    ret |= ACOUSTIC_EC_PTR_CHANNELS_ERROR;
  }

  if (pHandler->ptr_reference_channels > 0)
  {
    echoInstance->ptr_reference_channels = pHandler->ptr_reference_channels;
  }
  else
  {
    echoInstance->ptr_reference_channels = 1;
    ret |= ACOUSTIC_EC_PTR_CHANNELS_ERROR;
  }

  if (pHandler->ptr_output_channels > 0)
  {
    echoInstance->ptr_output_channels = pHandler->ptr_output_channels;
  }
  else
  {
    echoInstance->ptr_output_channels = 1;
    ret |= ACOUSTIC_EC_PTR_CHANNELS_ERROR;
  }

  if (pHandler->tail_length > 0)
  {
    echoInstance->tail_length = pHandler->tail_length;
  }
  else
  {
    echoInstance->tail_length = 512;
    ret |= ACOUSTIC_EC_TAIL_LENGTH_ERROR;
  }

  uint32_t byte_offset = sizeof(InternalEchoType);
  uint8_t *pByte = (uint8_t *)pHandler->pInternalMemory;
  uint32_t n = (((echoInstance->tail_length + NN_MAX - 1) / NN_MAX) + 1) * NN_MAX * 2;

  /*CHECK MEMORY alloc*/
  echoInstance->st.X = (float *)(pByte + byte_offset);
  byte_offset += n * sizeof(float);
  echoInstance->st.W = (float *)(pByte + byte_offset);
  byte_offset += n * sizeof(float);
  echoInstance->st.foreground = (float *)(pByte + byte_offset);
  byte_offset += n * sizeof(float);
  echoInstance->st.prop = (float *)(pByte + byte_offset);
  byte_offset += echoInstance->tail_length * sizeof(float);

  echoInstance->samples_count_output = ECHO_BUFF;
  echoInstance->buffer_state = 0;
  echoInstance->ECHO = 1;
  uint16_t sampleRate = FS;

  echoInstance->st.fft_table = &echoInstance->table;
  Echo_init(&echoInstance->st, NN, pHandler->tail_length, 1, 1);
  Echo_ctrl(&echoInstance->st, SPEEX_ECHO_SET_SAMPLING_RATE, &sampleRate);
  libSpeexAEC_fft_init(&echoInstance->table, (int)NN_MAX * 2);

  if (echoInstance->preprocess_initialized == 1)
  {
    echoInstance->PREPROCESS = 1;

    echoInstance->den = (SpeexPreprocessState *)(pByte + byte_offset);
    byte_offset += sizeof(SpeexPreprocessState);
    echoInstance->table_den = (drft_lookup *)(pByte + byte_offset);
    byte_offset += sizeof(drft_lookup);
    echoInstance->filterBank = (FilterBank *)(pByte + byte_offset);
    byte_offset += sizeof(FilterBank);

    libSpeexAEC_fft_init(echoInstance->table_den, (int)NN_MAX * 2);
    echoInstance->den->fft_lookup = echoInstance->table_den;
    Preprocess_init(echoInstance->den, NN, sampleRate);
    filterbank_new(echoInstance->filterBank, NB_BANDS, sampleRate, NN_MAX, 1);
    echoInstance->den->bank = echoInstance->filterBank;
  }

  return (pHandler->internal_memory_size != byte_offset);
}


uint32_t libSpeexAEC_getMemorySize(AcousticEC_Handler_t *pHandler)
{
  uint32_t ret = 0;
  uint32_t byte_offset = sizeof(InternalEchoType);
  uint32_t n = (((pHandler->tail_length + NN_MAX - 1) / NN_MAX) + 1) * NN_MAX * 2;

  byte_offset += n * sizeof(float);
  byte_offset += n * sizeof(float);
  byte_offset += n * sizeof(float);
  byte_offset += (pHandler->tail_length) * sizeof(float);

  if (pHandler->preprocess_init == 1)
  {
    byte_offset += sizeof(SpeexPreprocessState);
    byte_offset += sizeof(drft_lookup);
    byte_offset += sizeof(FilterBank);
  }

  pHandler->internal_memory_size = byte_offset;

  return ret;
}

/**
* @brief Library run function, performs audio analysis when all required data has been collected
* @retval 0 if everything is ok, 1 otherwise
*/
uint32_t libSpeexAEC_setConfig(AcousticEC_Handler_t *pHandler, AcousticEC_Config_t *pConfig)
{
  InternalEchoType *EchoInternal = (InternalEchoType *)(pHandler->pInternalMemory);
  uint32_t ret = 0;

  if (EchoInternal->preprocess_initialized == 1)
  {
    /*PREPROCESS*/
    if ((pConfig->preprocess_state & ACOUSTIC_EC_PREPROCESS_ENABLE) != 0U)
    {
      EchoInternal->PREPROCESS = 1;
    }
    else
    {
      EchoInternal->PREPROCESS = 0;
    }

    /*AGC*/
    if ((pConfig->AGC_value > 0) && (pConfig->AGC_value < 32768))
    {
      EchoInternal->den->agc_enabled = 1;
      EchoInternal->den->agc_level = pConfig->AGC_value;
    }
    else if (pConfig->AGC_value == 0)
    {
      EchoInternal->den->agc_enabled = 0;
    }
    else
    {
      EchoInternal->den->agc_enabled = 0;
      ret |= ACOUSTIC_EC_AEC_LEVEL_ERROR;
    }

    EchoInternal->den->noise_suppress = pConfig->noise_suppress_default;
    EchoInternal->den->echo_suppress = pConfig->echo_suppress_default;
    EchoInternal->den->echo_suppress_active = pConfig->echo_suppress_active;

    if (pConfig->residual_echo_remove == 1)
    {
      Preprocess_setup(EchoInternal->den, SPEEX_PREPROCESS_SET_ECHO_STATE, &EchoInternal->st);
    }
    else
    {
      Preprocess_setup(EchoInternal->den, SPEEX_PREPROCESS_SET_ECHO_STATE, 0);
    }
  }
  else
  {
    /*PREPROCESS*/
    if ((pConfig->preprocess_state & ACOUSTIC_EC_PREPROCESS_ENABLE) != 0U)
    {
      ret |= ACOUSTIC_EC_PREPROCESS_ERROR;
    }
  }
  return ret;
}

/**
* @brief Library run function, performs audio analisys when  all required data has been collected
* @retval 0 if everything is ok, 1 otherwise
*/
uint32_t libSpeexAEC_getConfig(AcousticEC_Handler_t *pHandler, AcousticEC_Config_t *pConfig)
{
  InternalEchoType *EchoInternal = (InternalEchoType *)(pHandler->pInternalMemory);
  uint32_t ret = 0;

  if (EchoInternal->preprocess_initialized == 1)
  {
    /*PREPROCESS*/
    pConfig->preprocess_state = EchoInternal->PREPROCESS;
    pConfig->AGC_value = (uint32_t)EchoInternal->den->agc_level;
  }
  else
  {
    ret |= ACOUSTIC_EC_PREPROCESS_ERROR;
  }
  return ret;
}

uint32_t libSpeexAEC_Process(AcousticEC_Handler_t *pHandler)
{
  InternalEchoType *echoInstance = (InternalEchoType *)(pHandler->pInternalMemory);
  uint32_t ret = 0;
  uint8_t ECHO = echoInstance->ECHO;
  uint8_t PREPROCESS = echoInstance->PREPROCESS;

  if (echoInstance->buffer_state == 1)
  {
    echoInstance->buffer_state = 0;
    if (((PREPROCESS && ECHO) == 0)  || ((PREPROCESS >= 1) && (ECHO == 0)))
    {
      uint32_t i = 0;
      for (i = 0; i < ECHO_BUFF; i++)
      {
        echoInstance->e_buf[ECHO_BUFF + i] = echoInstance->dir1_buf[i];
      }
    }
    if (ECHO)
    {
      Echo_cancellation(&echoInstance->st, echoInstance->dir1_buf, echoInstance->dir2_buf, echoInstance->e_buf + ECHO_BUFF);
    }
    if (PREPROCESS)
    {
      Preprocess(echoInstance->den, echoInstance->e_buf + ECHO_BUFF);
    }
  }
  else if (echoInstance->buffer_state == 2)
  {
    echoInstance->buffer_state = 0;
    if ((ECHO == 0) || ((PREPROCESS >= 1) && (ECHO == 0)))
    {
      uint32_t i = 0;
      for (i = 0; i < ECHO_BUFF; i++)
      {
        echoInstance->e_buf[i] = echoInstance->dir1_buf[i + ECHO_BUFF];
      }
    }
    if (ECHO)
    {
      Echo_cancellation(&echoInstance->st, echoInstance->dir1_buf + ECHO_BUFF, echoInstance->dir2_buf + ECHO_BUFF, echoInstance->e_buf);
    }
    if (PREPROCESS)
    {
      Preprocess(echoInstance->den, echoInstance->e_buf);
    }
  }
  else
  {
    /* do nothing: MISRAC */
  }
  return ret;
}

uint32_t libSpeexAEC_Data_Input(void *ptrPrimary, void *ptrReference, void *ptrBufferOut, AcousticEC_Handler_t *pHandler)
{
  InternalEchoType *echoInstance = (InternalEchoType *)(pHandler->pInternalMemory);
  uint8_t ret = 0;
  uint16_t i = 0;

  for (i = 0; i < (FS / 1000); i++)
  {
    /* Input */
    echoInstance->dir1_buf[echoInstance->samples_count] = ((int16_t *)ptrPrimary)[i * echoInstance->ptr_primary_channels];
    echoInstance->dir2_buf[echoInstance->samples_count] = ((int16_t *)ptrReference)[i * echoInstance->ptr_reference_channels];
    echoInstance->samples_count++;
    if (echoInstance->samples_count == ECHO_BUFF)
    {
      echoInstance->buffer_state = 1;
      ret = ACOUSTIC_EC_TAIL_LENGTH_ERROR;
    }
    else if (echoInstance->samples_count == (ECHO_BUFF * 2))
    {
      echoInstance->buffer_state = 2;
      echoInstance->samples_count = 0;
      ret = ACOUSTIC_EC_TAIL_LENGTH_ERROR;
    }
    else
    {
      /* do nothing: MISRAC */
    }

    /* Output */
    ((int16_t *)ptrBufferOut)[i * echoInstance->ptr_output_channels] = echoInstance->e_buf[echoInstance->samples_count_output];
    echoInstance->samples_count_output++;
    if (echoInstance->samples_count_output == (ECHO_BUFF * 2))
    {
      echoInstance->samples_count_output = 0;
    }
  }
  return ret;
}

void libSpeexAEC_fft_init(drft_lookup *table, int32_t size)
{
  drft_init(table, size);
}

void libSpeexAEC_fft(drft_lookup *table, float32_t *in, float32_t *out)
{
  int32_t i;
  float32_t scale = 1.0f / ((float32_t)(table->n));
  for (i = 0; i < table->n; i++)
  {
    out[i] = scale * in[i];
  }
  drft_forward(table, out);
}

void libSpeexAEC_ifft(drft_lookup *table, float32_t *in, float32_t *out)
{
  int32_t i;
  for (i = 0; i < table->n; i++)
  {
    out[i] = in[i];
  }
  drft_backward(table, out);
}

