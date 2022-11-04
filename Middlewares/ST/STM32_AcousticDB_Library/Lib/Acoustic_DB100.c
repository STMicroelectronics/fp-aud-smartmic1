/**
******************************************************************************
* @file    Acoustic_db100.c
* @author  Central Labs
* @version V1.0.0
* @date    01-Sep-2016
* @brief   This file contains dB SPL library functions.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Some of the library code is based on the CMSIS DSP software library by ARM,
* a suite of common signal processing functions for use on Cortex-M processor
* based devices. Licencing terms are available in the attached release_note.html
* file, in the next lines of this
* document and it's available on the web at:
* http://www.keil.com/pack/doc/CMSIS/DSP/html/index.html
*
*   ARM licence note:
*
* Copyright (C) 2009-2012 ARM Limited.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*  - Neither the name of ARM nor the names of its contributors may be used
*   to endorse or promote products derived from this software without
*   specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "acoustic_db.h"
#include "math.h"
#include "arm_math.h"

/* Private typedef -----------------------------------------------------------*/

typedef struct {
  uint16_t sampling_frequency; 
  uint16_t samples_to_process;
  uint16_t Input_Counter;
  int16_t offset;
  uint64_t first_accumulator;
  uint64_t second_accumulator;
  uint32_t data_ready;
} libdBNoise_Handler_Internal;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint32_t AcousticDB_Init(AcousticDB_Handler_t * pHandler)
{
  libdBNoise_Handler_Internal * dBInternal = (libdBNoise_Handler_Internal *)(pHandler->pInternalMemory);
  uint32_t ret = 0;   
  
  /* FS */
  if(pHandler->sampling_frequency == 16000 || pHandler->sampling_frequency == 32000 || pHandler->sampling_frequency == 48000 || pHandler->sampling_frequency == 8000)
  {
    dBInternal->sampling_frequency=pHandler->sampling_frequency;
  }
  else
  {
    dBInternal->sampling_frequency=16000; /*Set default Value*/
    ret |= ACOUSTIC_DB_SAMPLING_FREQ_ERROR;
  }
  
  dBInternal->samples_to_process = (dBInternal->sampling_frequency/1000)*64;
  dBInternal->Input_Counter=0;
  dBInternal->first_accumulator=0;
  dBInternal->second_accumulator=0;
  dBInternal->offset=0;
  dBInternal->data_ready=0;
  return ret;
}

uint32_t AcousticDB_Data_Input(void *pInput, uint32_t nSamples, AcousticDB_Handler_t * pHandler)
{  
  libdBNoise_Handler_Internal * dBInternal = (libdBNoise_Handler_Internal *)(pHandler->pInternalMemory);    
  
  uint16_t i = 0;
  uint32_t ret = 0;
  uint16_t  samples_to_process = (dBInternal->samples_to_process);
  uint16_t  input_counter = (dBInternal->Input_Counter);
  
  for (i = 0; i < nSamples/2; i++)
  {
    if( input_counter < samples_to_process)
    {
      dBInternal->first_accumulator = __SMLALD(((uint32_t *)(pInput))[i], ((uint32_t *)(pInput))[i], dBInternal->first_accumulator);    
      input_counter+=2;      
      if(input_counter == samples_to_process)
      {
        dBInternal->data_ready=1;
        ret = 1;
      }
    }
    else 
    {      
      dBInternal->second_accumulator = __SMLALD(((uint32_t *)(pInput))[i], ((uint32_t *)(pInput))[i],dBInternal->second_accumulator); 
      input_counter+=2;
      if(input_counter == (2 * samples_to_process))
      {
        input_counter = 0;
        dBInternal->data_ready = 2;
        ret = 1;
      }
    }    
  }
  dBInternal->Input_Counter =  input_counter;
  return ret; 
}


uint32_t AcousticDB_Process(int32_t * dB_Value, AcousticDB_Handler_t * pHandler)
{
  libdBNoise_Handler_Internal * dBInternal = (libdBNoise_Handler_Internal *)(pHandler->pInternalMemory);  
  static uint8_t LT[100]=
{
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 4,
  4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9,
  9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10,
  10, 10, 10, 10, 10};
  
  uint32_t avg = 0, rem = 0, res = 0;
  
  if(dBInternal->data_ready == 1)
  {
    avg = (dBInternal->first_accumulator / dBInternal->samples_to_process);
    dBInternal->first_accumulator=0;
  }
  else if(dBInternal->data_ready == 2)
  {
    avg = (dBInternal->second_accumulator / dBInternal->samples_to_process);
    dBInternal->second_accumulator=0;
  }
  
  if (avg >= (uint32_t)1000)
  {  
    if (avg >= (uint32_t)1000000)  
    {      
      if(avg >= (uint32_t)100000000)
      {  
        if(avg >= (uint32_t)1000000000)
        {
          res = 90;
          rem = avg/100000000;  
        }
        else
        {
          res = 80;
          rem = avg/10000000; 
        }        
      }
      else
      {
        if(avg >= (uint32_t)10000000)
        {  
          res = 70;
          rem = avg/1000000; 
        }
        else
        {
          res = 60;
          rem = avg/100000; 
        }
      }      
    }
    else  
    {
      if (avg >= (uint32_t)10000)  
      {
        if (avg >= (uint32_t)100000)  
        {
          res = 50;
          rem = avg/10000; 
        }
        else 
        {      
          res = 40;
          rem = avg/1000; 
        }       
      }
      else  
      {
        res = 30;
        rem = avg/100; 
      }     
    }
  }
  else
  {
    if (avg >= (uint32_t)10)
    {
      if (avg >= (uint32_t)100)  
      {   
        res = 20;
        rem = avg/10; 
      }
      else  
      {
        res = 10;
        rem = avg; 
      }
    }
    else   
    {
      rem = avg*10; 
    }
  }  
  
  res += 30 + LT[rem];  
  dBInternal->data_ready = 0;  
  *dB_Value = (int32_t)res + dBInternal->offset;   
  return 0;
}

uint32_t AcousticDB_setConfig(AcousticDB_Handler_t * pHandler, AcousticDB_Config_t * pConfig)
{
  libdBNoise_Handler_Internal * dBInternal = (libdBNoise_Handler_Internal *)(pHandler->pInternalMemory);
  dBInternal->offset = pConfig->offset;
  uint32_t ret = 0;
  return ret;
}

uint32_t AcousticDB_getConfig(AcousticDB_Handler_t * pHandler, AcousticDB_Config_t * pConfig)
{
  libdBNoise_Handler_Internal * dBInternal = (libdBNoise_Handler_Internal *)(pHandler->pInternalMemory);
  pConfig->offset = dBInternal->offset;
  return 0;  
}

uint32_t AcousticDB_GetLibVersion(char *version)
{  
  char str1[35] = "ST AcousticDB v1.0.0";  
  strcpy(version, str1); 
  return strlen(str1);
}
