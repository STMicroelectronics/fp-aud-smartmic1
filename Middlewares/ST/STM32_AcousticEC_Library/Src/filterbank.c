/**
******************************************************************************
* @file    filterbank.c
* @author  SRA
* @brief   Utilities to convert between psd and filterbank
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
#include <arm_math.h>

#include "defines_ec.h"

#define toBARK(n)   ((13.1f*atan(.00074f*(n))) + (2.24f*atan((n)*(n)*1.85e-8f)) + (1e-4f*(n)))
#define toMEL(n)    (2595.f*log10(1.f+(n)/700.f))


void filterbank_new(FilterBank *bank, int32_t banks, spx_word32_t sampling, int32_t len, int32_t type)
{
  //UNUSED(type);
  int32_t i;
  int32_t id1;
  int32_t id2;
  spx_word32_t df;
  spx_word32_t max_mel, mel_interval;

  df = DIV32(SHL32(sampling, 15), MULT16_16(2, len));
  max_mel = (float32_t)(toBARK((float64_t)(EXTRACT16((float64_t)sampling / 2.0f))));
  mel_interval = PDIV32(max_mel, (float32_t)banks - 1.0f);
  bank->nb_banks = banks;
  bank->len = len;

  for (i = 0; i < len; i++)
  {
    spx_word16_t curr_freq;
    spx_word32_t mel;
    spx_word16_t val;
    curr_freq = EXTRACT16(MULT16_32_P15((float32_t)i, df));
    mel = (float32_t)(toBARK((float64_t)curr_freq));

    if (mel > max_mel)
    {
      break;
    }
    id1 = (int32_t)(floor((float64_t)mel / (float64_t)mel_interval));

    if (id1 > (banks - 2))
    {
      id1 = banks - 2;
      val = Q15_ONE;
    }
    else
    {
      val = DIV32_16(mel - ((float32_t)id1 * mel_interval), EXTRACT16(PSHR32(mel_interval, 15)));
    }

    id2 = id1 + 1;
    bank->bank_left[i] = id1;
    bank->filter_left[i] = SUB16(Q15_ONE, val);
    bank->bank_right[i] = id2;
    bank->filter_right[i] = val;
  }

  for (i = 0; i < bank->nb_banks; i++)
  {
    bank->scaling[i] = 0.0f;
  }

  for (i = 0; i < bank->len; i++)
  {
    int32_t id = bank->bank_left[i];
    bank->scaling[id] += bank->filter_left[i];
    id = bank->bank_right[i];
    bank->scaling[id] += bank->filter_right[i];
  }

  for (i = 0; i < bank->nb_banks; i++)
  {
    bank->scaling[i] = Q15_ONE / (bank->scaling[i]);
  }
}

void filterbank_compute_bank32(FilterBank *bank, spx_word32_t *ps, spx_word32_t *mel)
{
  int32_t i;
  for (i = 0; i < bank->nb_banks; i++)
  {
    mel[i] = 0.0f;
  }

  for (i = 0; i < bank->len; i++)
  {
    int32_t id;
    id = bank->bank_left[i];
    mel[id] += MULT16_32_P15(bank->filter_left[i], ps[i]);
    id = bank->bank_right[i];
    mel[id] += MULT16_32_P15(bank->filter_right[i], ps[i]);
  }
}

void filterbank_compute_psd16(FilterBank *bank, spx_word16_t *mel, spx_word16_t *ps)
{
  int32_t i;
  for (i = 0; i < bank->len; i++)
  {
    spx_word32_t tmp;
    int32_t id1, id2;
    id1 = bank->bank_left[i];
    id2 = bank->bank_right[i];
    tmp = MULT16_16(mel[id1], bank->filter_left[i]);
    tmp += MULT16_16(mel[id2], bank->filter_right[i]);
    ps[i] = EXTRACT16(PSHR32(tmp, 15));
  }
}

