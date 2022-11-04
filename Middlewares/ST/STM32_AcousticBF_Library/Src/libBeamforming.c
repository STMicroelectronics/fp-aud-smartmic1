/**
******************************************************************************
* @file    libBeamforming.c
* @author  SRA
* @brief   Beamforming core library
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
#ifndef __LIB_BEAMFORMING_C
#define __LIB_BEAMFORMING_C

/* Includes ------------------------------------------------------------------*/
#include "acoustic_bf.h"
#include "defines.h"

#include "filterbank.c"
#include "adaptive.c"
#include "denoiser.c"
#include "smallft.c"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#define SPEED_OF_SOUND 343.0f

/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

static uint32_t libBeamforming_GetLibVersion(char *version)
{
  char str1[35] = "ST AcousticBF v3.0.1";
  
  (void)strcpy(version, str1);
  return strlen(str1);
}

static uint32_t libBeamforming_Init(AcousticBF_Handler_t * pHandler)
{
  libBeamforming_Handler_Internal *BeamFormingInternal = (libBeamforming_Handler_Internal *)(pHandler->pInternalMemory);
 (void)memset(pHandler->pInternalMemory, 0 , pHandler->internal_memory_size);
  
  uint32_t ret = 0; 
  /*SUPPORT VARIABLE USED FOR MEMORY ALLOCATION*/  
  volatile uint32_t  byte_offset = sizeof(libBeamforming_Handler_Internal); 
  
  BeamFormingInternal->Buffer_State=0;
  BeamFormingInternal->Samples_Count=0;
  BeamFormingInternal->Samples_Count_Output=INTERNAL_BUFF_SIZE/2U;
  
  BeamFormingInternal->Dir1_Buffer = (int16_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
  byte_offset+=INTERNAL_BUFF_SIZE*sizeof(int16_t);  
  
  BeamFormingInternal->Dir2_Buffer = (int16_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
  byte_offset+=INTERNAL_BUFF_SIZE*sizeof(int16_t); 
  
  BeamFormingInternal->E_Buffer = (int16_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
  byte_offset+=INTERNAL_BUFF_SIZE*sizeof(int16_t); 
  
  if(pHandler->algorithm_type_init <= ACOUSTIC_BF_TYPE_STRONG)
  {
    BeamFormingInternal->algorithm_type_init = pHandler->algorithm_type_init;
  }
  else
  {
    BeamFormingInternal->algorithm_type_init = ACOUSTIC_BF_TYPE_CARDIOID_BASIC;
    ret |= ACOUSTIC_BF_TYPE_ERROR;
  }
  
  if(pHandler->ptr_out_channels>0U)
  {
    BeamFormingInternal->ptr_out_channels = pHandler->ptr_out_channels;
  }
  else
  {
    BeamFormingInternal->ptr_out_channels = 2;
    ret |= ACOUSTIC_BF_PTR_CHANNELS_ERROR;
  }
  
  
  if((pHandler->ref_mic_enable == ACOUSTIC_BF_REF_ENABLE) || (pHandler->ref_mic_enable == ACOUSTIC_BF_REF_DISABLE) || (pHandler->ref_mic_enable == 7U))
  {
    BeamFormingInternal->ref_mic_enable = pHandler->ref_mic_enable;
  }
  else
  {
    BeamFormingInternal->ref_mic_enable = 1;
    ret |= ACOUSTIC_BF_REF_OUT_ERROR;
  }
  
  BeamFormingInternal->gain_0=1.0f;
  BeamFormingInternal->gain_old_0=1.0f;
  
  /*************NONE*****************/
  /*interleaving mic 1*/
  if(pHandler->ptr_M1_channels>0U)
  {
    BeamFormingInternal->ptr_M1_channels = pHandler->ptr_M1_channels;
  }
  else
  {
    BeamFormingInternal->ptr_M1_channels = 2;
    ret |= ACOUSTIC_BF_PTR_CHANNELS_ERROR;
  }
  
  /*interleaving mic 2*/
  if(pHandler->ptr_M2_channels>0U)
  {
    BeamFormingInternal->ptr_M2_channels = pHandler->ptr_M2_channels;
  }
  else
  {
    BeamFormingInternal->ptr_M2_channels = 2;
    ret |= ACOUSTIC_BF_PTR_CHANNELS_ERROR;
  }
  
  /*sampling frequency*/
  if((pHandler->sampling_frequency == 16U) || (pHandler->sampling_frequency == 768U) || (pHandler->sampling_frequency == 1024U) || (pHandler->sampling_frequency == 256U) || (pHandler->sampling_frequency == 512U) || (pHandler->sampling_frequency == 384U) || (pHandler->sampling_frequency == 1280U) || (pHandler->sampling_frequency == 2048U) )
  {
    BeamFormingInternal->sampling_frequency = pHandler->sampling_frequency;
  }
  else
  {
    BeamFormingInternal->sampling_frequency = 1024;
    ret |= ACOUSTIC_BF_SAMPLING_FREQ_ERROR;
  }  
  
  /*Data Type PDM or PCM*/
  if((pHandler->data_format == ACOUSTIC_BF_DATA_FORMAT_PDM) || (pHandler->data_format == ACOUSTIC_BF_DATA_FORMAT_PCM))
  {
    BeamFormingInternal->data_format = pHandler->data_format;    
  }
  else
  {
    BeamFormingInternal->data_format = ACOUSTIC_BF_DATA_FORMAT_PDM;
    ret |= ACOUSTIC_BF_DATA_FORMAT_ERROR;
  } 
  
  /*Delay performed*/
  if((BeamFormingInternal->data_format == ACOUSTIC_BF_DATA_FORMAT_PDM) && (pHandler->delay_enable == 1U))
  {
    BeamFormingInternal->delay_enable = pHandler->delay_enable;    
  }
  else if((BeamFormingInternal->data_format == ACOUSTIC_BF_DATA_FORMAT_PDM) && (pHandler->delay_enable == 0U)) 
  {
    BeamFormingInternal->delay_enable = 1;
    ret |= ACOUSTIC_BF_DELAY_ERROR;
  } 
  else if(BeamFormingInternal->data_format == ACOUSTIC_BF_DATA_FORMAT_PCM ) 
  {
    BeamFormingInternal->delay_enable = pHandler->delay_enable;
  }           
  else
  {
    ret |= ACOUSTIC_BF_DELAY_ERROR;
  }
  
  
  if(BeamFormingInternal->data_format == ACOUSTIC_BF_DATA_FORMAT_PDM)
  {
    BeamFormingInternal->Filter_M1_Standard_Handler = (PDM_Filter_Handler_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(PDM_Filter_Handler_t);      
    
    BeamFormingInternal->Filter_M2_Delayed_Handler = (PDM_Filter_Handler_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(PDM_Filter_Handler_t);
    
    BeamFormingInternal->Filter_M2_Standard_Handler = (PDM_Filter_Handler_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(PDM_Filter_Handler_t);
    
    BeamFormingInternal->Filter_M1_Delayed_Handler = (PDM_Filter_Handler_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(PDM_Filter_Handler_t);
    
    BeamFormingInternal->Filter_M1_Standard_Config = (PDM_Filter_Config_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(PDM_Filter_Config_t);      
    
    BeamFormingInternal->Filter_M2_Delayed_Config = (PDM_Filter_Config_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(PDM_Filter_Config_t);
    
    BeamFormingInternal->Filter_M2_Standard_Config = (PDM_Filter_Config_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(PDM_Filter_Config_t);
    
    BeamFormingInternal->Filter_M1_Delayed_Config = (PDM_Filter_Config_t *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(PDM_Filter_Config_t);  
  }
  
  if(BeamFormingInternal->delay_enable == 1U)
  {
    BeamFormingInternal->Delay_M2 = (TDelay *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(TDelay);
    
    BeamFormingInternal->Delay_M1 = (TDelay *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(TDelay); 
  }
  
  BeamFormingInternal->BFParam_1 = (TPCMBF *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
  byte_offset+=sizeof(TPCMBF);  
  
  BeamFormingInternal->BFParam_2 = (TPCMBF *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
  byte_offset+=sizeof(TPCMBF);   
  
  
  /********** DENOISER (FOR LIGHT OR STRONG VERSIONS)********/
  if((BeamFormingInternal->algorithm_type_init == ACOUSTIC_BF_TYPE_CARDIOID_DENOISE) || (BeamFormingInternal->algorithm_type_init == ACOUSTIC_BF_TYPE_STRONG))
  { 
    BeamFormingInternal->den = (SpeexPreprocessState *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(SpeexPreprocessState); 
    BeamFormingInternal->table_den = (drft_lookup *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(drft_lookup);
    BeamFormingInternal->filterBank = (FilterBank *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(FilterBank);
    fft_init(BeamFormingInternal->table_den, (int32_t)NN_MAX*2);
    BeamFormingInternal->den->fft_lookup=(drft_lookup *)BeamFormingInternal->table_den;
    denoiserstate_init((SpeexPreprocessState *)BeamFormingInternal->den,NN, 16000); 
    filterbank_new((FilterBank *) BeamFormingInternal->filterBank,NB_BANDS,16000.0f,NN_MAX,1);
    BeamFormingInternal->den->bank=(FilterBank *) BeamFormingInternal->filterBank;
  }
  
  /********** ADAPTIVE (FOR ASR OR STRONG VERSIONS)********/
  if((BeamFormingInternal->algorithm_type_init == ACOUSTIC_BF_TYPE_ASR_READY) || (BeamFormingInternal->algorithm_type_init == ACOUSTIC_BF_TYPE_STRONG))
  { 
    BeamFormingInternal->st = (SpeexEchoState *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(SpeexEchoState); 
    BeamFormingInternal->table = (drft_lookup *)((uint8_t *)pHandler->pInternalMemory + byte_offset);
    byte_offset+=sizeof(drft_lookup); 
    fft_init(BeamFormingInternal->table, (int32_t)NN_MAX*2);
    BeamFormingInternal->st->fft_table=BeamFormingInternal->table;
    adaptivestate_init_mc((SpeexEchoState *) BeamFormingInternal->st, NN, TAIL, 1, 1); 
  }
  
  if(BeamFormingInternal->algorithm_type_init == ACOUSTIC_BF_TYPE_STRONG)
  {     
    (void)denoiser_setup((SpeexPreprocessState *)BeamFormingInternal->den, SPEEX_PREPROCESS_SET_ECHO_STATE, (SpeexEchoState *)BeamFormingInternal->st);
  } 
  /*Optimization track to allow switching between modes if everything is initialized*/
  BeamFormingInternal->initialized_as = (uint16_t)BeamFormingInternal->algorithm_type_init; 
  return ret;
}

static uint32_t libBeamforming_FirstStep(void *pM1, void *pM2, void *ptr_Out, AcousticBF_Handler_t * pHandler)
{
  libBeamforming_Handler_Internal * BeamFormingInternal = (libBeamforming_Handler_Internal *)(pHandler->pInternalMemory);  
  return BeamFormingInternal->Callbacks.FirstStep(pM1,pM2, ptr_Out, BeamFormingInternal);
}

static uint32_t libBeamforming_SecondStep(AcousticBF_Handler_t * pHandler)
{   
  uint32_t ret = 0;
  libBeamforming_Handler_Internal * BeamFormingInternal = (libBeamforming_Handler_Internal *)(pHandler->pInternalMemory); 
  
  uint32_t level = BeamFormingInternal->algorithm_type_init;
  if(BeamFormingInternal->Buffer_State==1U)
  {
    BeamFormingInternal->Buffer_State=0;    
    
    if((level==ACOUSTIC_BF_TYPE_CARDIOID_BASIC) || (level==ACOUSTIC_BF_TYPE_CARDIOID_DENOISE))
    {
      int32_t i;
      for(i=0;i<128;i++)
      {
        BeamFormingInternal->E_Buffer[128+i]=BeamFormingInternal->Dir1_Buffer[i];
      }
    }       
    if((level==ACOUSTIC_BF_TYPE_STRONG) || (level==ACOUSTIC_BF_TYPE_ASR_READY))
    {
      (void)adaptive_A_run((SpeexEchoState *)BeamFormingInternal->st, (int16_t *)BeamFormingInternal->Dir1_Buffer, (int16_t *)BeamFormingInternal->Dir2_Buffer, (int16_t *)(BeamFormingInternal->E_Buffer+128));
    }
    
    if((level==ACOUSTIC_BF_TYPE_STRONG) || (level==ACOUSTIC_BF_TYPE_CARDIOID_DENOISE))
    {
      (void)denoiser_A_run((SpeexPreprocessState *)BeamFormingInternal->den, (int16_t *)(BeamFormingInternal->E_Buffer+128));
    }
  }
  else if(BeamFormingInternal->Buffer_State==2U)
  {
    BeamFormingInternal->Buffer_State=0;    
    
    if((level==ACOUSTIC_BF_TYPE_CARDIOID_BASIC) || (level==ACOUSTIC_BF_TYPE_CARDIOID_DENOISE))
    {
      int32_t i;
      for(i=0;i<128;i++)
      {
        BeamFormingInternal->E_Buffer[i]=BeamFormingInternal->Dir1_Buffer[i + 128];
      }      
    } 
    if((level==ACOUSTIC_BF_TYPE_STRONG) || (level==ACOUSTIC_BF_TYPE_ASR_READY))
    {
      (void)adaptive_A_run((SpeexEchoState *)BeamFormingInternal->st, (int16_t *)(BeamFormingInternal->Dir1_Buffer+128), (int16_t *)(BeamFormingInternal->Dir2_Buffer+128), (int16_t *)(BeamFormingInternal->E_Buffer));
    }
    
    if((level==ACOUSTIC_BF_TYPE_STRONG) || (level==ACOUSTIC_BF_TYPE_CARDIOID_DENOISE))
    {
      (void)denoiser_A_run((SpeexPreprocessState *)BeamFormingInternal->den, (int16_t *)(BeamFormingInternal->E_Buffer));
    }
  }
  else
  {
    ret = ACOUSTIC_BF_PROCESSING_ERROR;
  }
  return ret;
}

/*Parameters Managed:
-Distance
-Level
-Gain
-samplingFrequency
*/
static uint32_t libBeamforming_setConfig(AcousticBF_Handler_t * pHandler, AcousticBF_Config_t * pConfig)
{
  libBeamforming_Handler_Internal * BeamFormingInternal = (libBeamforming_Handler_Internal *)(pHandler->pInternalMemory);  
  
  uint32_t dist_has_changed;
  uint32_t ret = 0;
  dist_has_changed = 0;
  
  if(pConfig->M2_gain >= 0.0f )
  {
    BeamFormingInternal->M2_gain = pConfig->M2_gain;
  }
  else
  {
    BeamFormingInternal->M2_gain = 1.0f;
    ret |= ACOUSTIC_BF_M2_GAIN_ERROR;
  } 
  
  /*TODO: ADD CONTROL HERE*/
  BeamFormingInternal->overall_gain = (uint8_t)pConfig->volume;
  
  if((pHandler->data_format == ACOUSTIC_BF_DATA_FORMAT_PCM) && (pHandler->delay_enable == 1U) && ( (pConfig->mic_distance <210U) || (pConfig->mic_distance >212U) ))
  {
    BeamFormingInternal->mic_distance = 212;
    ret |= ACOUSTIC_BF_DISTANCE_ERROR;
  } 
  else
  {
    if(pConfig->mic_distance != BeamFormingInternal->mic_distance)
    {
      dist_has_changed=1;
      if((pConfig->mic_distance > 0U) && (pConfig->mic_distance <= 212U))
      {
        BeamFormingInternal->mic_distance = pConfig->mic_distance;   
      }
      else
      {
        BeamFormingInternal->mic_distance = 150;
        ret |= ACOUSTIC_BF_DISTANCE_ERROR;
      }
    }
  }
  
  if((BeamFormingInternal->initialized_as == ACOUSTIC_BF_TYPE_STRONG) || (BeamFormingInternal->initialized_as == ACOUSTIC_BF_TYPE_ASR_READY))
  {
    BeamFormingInternal->Filter_M1_Standard_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
    (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M1_Standard_Handler, BeamFormingInternal->Filter_M1_Standard_Config);
    
    BeamFormingInternal->Filter_M2_Standard_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
    (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M2_Standard_Handler, BeamFormingInternal->Filter_M2_Standard_Config);
    
    BeamFormingInternal->Filter_M1_Delayed_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
    (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M1_Delayed_Handler, BeamFormingInternal->Filter_M1_Delayed_Config);
    
    BeamFormingInternal->Filter_M2_Delayed_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
    (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M2_Delayed_Handler, BeamFormingInternal->Filter_M2_Delayed_Config);
  }
  else
  {
    BeamFormingInternal->Filter_M1_Standard_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
    (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M1_Standard_Handler, BeamFormingInternal->Filter_M1_Standard_Config);
    
    BeamFormingInternal->Filter_M2_Delayed_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
    (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M2_Delayed_Handler, BeamFormingInternal->Filter_M2_Delayed_Config);
  }
  
  if((BeamFormingInternal->initialized_as == ACOUSTIC_BF_TYPE_STRONG)||(BeamFormingInternal->initialized_as==pConfig->algorithm_type)||(pConfig->algorithm_type==ACOUSTIC_BF_TYPE_CARDIOID_BASIC))
  {
    if((dist_has_changed != 0U) ||  (BeamFormingInternal->algorithm_type_init != pConfig->algorithm_type))
    {
      BeamFormingInternal->algorithm_type_init = pConfig->algorithm_type;
      /*CASE PDM DATA INPUT*/
      if(BeamFormingInternal->data_format == ACOUSTIC_BF_DATA_FORMAT_PDM)
      { 
        uint16_t dec_factor_temp = 0;
        
        switch (BeamFormingInternal->sampling_frequency / 16U )
        {
        case 16:
            dec_factor_temp = PDM_FILTER_DEC_FACTOR_16;
            break;
        case 32:
            dec_factor_temp = PDM_FILTER_DEC_FACTOR_32;
            break;
        case 48:
            dec_factor_temp = PDM_FILTER_DEC_FACTOR_48;
            break;
        case 64:
            dec_factor_temp = PDM_FILTER_DEC_FACTOR_64;
            break;
        case 80:
            dec_factor_temp = PDM_FILTER_DEC_FACTOR_80;
            break;
        case 128:
            dec_factor_temp = PDM_FILTER_DEC_FACTOR_128;
            break;
        default:
            break;
        }
        /*PDM to PCM, BF, Delay structure allocation for 1 cardioid case (NONE OR LIGHT) -> always */
        if(BeamFormingInternal->mic_distance<210U)
        {
          /*PDM INPUT, DELAY IN PDM THEN PDM TO PCM*/
          BeamFormingInternal->Filter_M1_Standard_Handler->bit_order = PDM_FILTER_BIT_ORDER_LSB;
          BeamFormingInternal->Filter_M1_Standard_Handler->endianness = PDM_FILTER_ENDIANNESS_BE;
          BeamFormingInternal->Filter_M1_Standard_Handler->high_pass_tap = 1932735282;
          BeamFormingInternal->Filter_M1_Standard_Handler->in_ptr_channels = BeamFormingInternal->ptr_M1_channels;
          BeamFormingInternal->Filter_M1_Standard_Handler->out_ptr_channels = 1;
          (void)PDM_Filter_Init(BeamFormingInternal->Filter_M1_Standard_Handler);
          
          BeamFormingInternal->Filter_M1_Standard_Config->output_samples_number = 16;
          BeamFormingInternal->Filter_M1_Standard_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
          BeamFormingInternal->Filter_M1_Standard_Config->decimation_factor = dec_factor_temp;    
          (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M1_Standard_Handler, BeamFormingInternal->Filter_M1_Standard_Config);
          
          BeamFormingInternal->Filter_M2_Delayed_Handler->bit_order = PDM_FILTER_BIT_ORDER_LSB;
          BeamFormingInternal->Filter_M2_Delayed_Handler->endianness = PDM_FILTER_ENDIANNESS_LE;
          BeamFormingInternal->Filter_M2_Delayed_Handler->high_pass_tap = 1932735282;
          BeamFormingInternal->Filter_M2_Delayed_Handler->in_ptr_channels = 1;
          BeamFormingInternal->Filter_M2_Delayed_Handler->out_ptr_channels = 1;
          (void)PDM_Filter_Init(BeamFormingInternal->Filter_M2_Delayed_Handler);
          
          BeamFormingInternal->Filter_M2_Delayed_Config->output_samples_number = 16;
          BeamFormingInternal->Filter_M2_Delayed_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
          BeamFormingInternal->Filter_M2_Delayed_Config->decimation_factor = dec_factor_temp;    
          (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M2_Delayed_Handler, BeamFormingInternal->Filter_M2_Delayed_Config);
          
          BeamFormingInternal->Delay_M2->jump_in=BeamFormingInternal->ptr_M2_channels;
          BeamFormingInternal->Delay_M2->len = (uint16_t)(BeamFormingInternal->sampling_frequency)/8U;
          BeamFormingInternal->Delay_M2->delay=(uint16_t)roundf(((float32_t)BeamFormingInternal->sampling_frequency*1000.0f)*((float32_t)BeamFormingInternal->mic_distance/10000.0f)/SPEED_OF_SOUND);
          Delay_Init(BeamFormingInternal->Delay_M2);
          
          if(BeamFormingInternal->Delay_M2->delay >= 64U)
          {
            BeamFormingInternal->Callbacks.Delay=Delay_3;    
          }
          else
          {
            BeamFormingInternal->Callbacks.Delay=Delay_2;
          }
          /*ONLY 1 FUNCTION BECAUSE OF THE PDM DELAY FUNCTION RETURNS ALWAYS 1 MIC WITHOUT INTERLEAVING*/
          BeamFormingInternal->Callbacks.PDM_Filter_Delayed=PDM_Filter; 
          BeamFormingInternal->Callbacks.PDM_Filter_Standard_M1=PDM_Filter;
          BeamFormingInternal->Callbacks.FirstStep=FirstStepInternal1;
        }
        else
        {  /*PDM INPUT, PDM TO PCM THEN DELAY SEEN THE DISTANCE IS >21 and <23 mm*/      
          BeamFormingInternal->Filter_M1_Standard_Handler->bit_order = PDM_FILTER_BIT_ORDER_LSB;
          BeamFormingInternal->Filter_M1_Standard_Handler->endianness = PDM_FILTER_ENDIANNESS_BE;
          BeamFormingInternal->Filter_M1_Standard_Handler->high_pass_tap = 1932735282;
          BeamFormingInternal->Filter_M1_Standard_Handler->in_ptr_channels = BeamFormingInternal->ptr_M1_channels;
          BeamFormingInternal->Filter_M1_Standard_Handler->out_ptr_channels = 1;
          (void)PDM_Filter_Init(BeamFormingInternal->Filter_M1_Standard_Handler);        
          BeamFormingInternal->Filter_M1_Standard_Config->output_samples_number = 16;
          BeamFormingInternal->Filter_M1_Standard_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
          BeamFormingInternal->Filter_M1_Standard_Config->decimation_factor = dec_factor_temp;    
          (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M1_Standard_Handler, BeamFormingInternal->Filter_M1_Standard_Config);
          
          BeamFormingInternal->Filter_M2_Standard_Handler->bit_order = PDM_FILTER_BIT_ORDER_LSB;
          BeamFormingInternal->Filter_M2_Standard_Handler->endianness = PDM_FILTER_ENDIANNESS_BE;
          BeamFormingInternal->Filter_M2_Standard_Handler->high_pass_tap = 1932735282;
          BeamFormingInternal->Filter_M2_Standard_Handler->in_ptr_channels = BeamFormingInternal->ptr_M2_channels;
          BeamFormingInternal->Filter_M2_Standard_Handler->out_ptr_channels = 1;
          (void)PDM_Filter_Init(BeamFormingInternal->Filter_M2_Standard_Handler);        
          BeamFormingInternal->Filter_M2_Standard_Config->output_samples_number = 16;
          BeamFormingInternal->Filter_M2_Standard_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
          BeamFormingInternal->Filter_M2_Standard_Config->decimation_factor = dec_factor_temp;    
          (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M2_Standard_Handler, BeamFormingInternal->Filter_M2_Standard_Config);
          
          BeamFormingInternal->Delay_M2->jump_in=1;
          BeamFormingInternal->Delay_M2->len=16;
          BeamFormingInternal->Delay_M2->delay=1;  
          Delay_Init(BeamFormingInternal->Delay_M2);
          BeamFormingInternal->Callbacks.Delay=Delay_1;
          
          /*TODO: optimize eventually*/
          BeamFormingInternal->Callbacks.PDM_Filter_Standard_M1=PDM_Filter;
          BeamFormingInternal->Callbacks.PDM_Filter_Standard_M2=PDM_Filter;
          
          BeamFormingInternal->Callbacks.FirstStep=FirstStepInternal3;
        }
        /*PDM to PCM, BF, Delay structure allocation for 2 cardioids case (ASR OR STRONG) -> only if needed */
        if(BeamFormingInternal->algorithm_type_init > ACOUSTIC_BF_TYPE_CARDIOID_DENOISE)
        {
          if(BeamFormingInternal->mic_distance<210U)
          {
            BeamFormingInternal->Filter_M2_Standard_Handler->bit_order = PDM_FILTER_BIT_ORDER_LSB;
            BeamFormingInternal->Filter_M2_Standard_Handler->endianness = PDM_FILTER_ENDIANNESS_BE;
            BeamFormingInternal->Filter_M2_Standard_Handler->high_pass_tap = 1932735282;
            BeamFormingInternal->Filter_M2_Standard_Handler->in_ptr_channels = BeamFormingInternal->ptr_M2_channels;
            BeamFormingInternal->Filter_M2_Standard_Handler->out_ptr_channels = 1;
            (void)PDM_Filter_Init(BeamFormingInternal->Filter_M2_Standard_Handler);        
            BeamFormingInternal->Filter_M2_Standard_Config->output_samples_number = 16;
            BeamFormingInternal->Filter_M2_Standard_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
            BeamFormingInternal->Filter_M2_Standard_Config->decimation_factor = dec_factor_temp;    
            (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M2_Standard_Handler, BeamFormingInternal->Filter_M2_Standard_Config);
            
            BeamFormingInternal->Filter_M1_Delayed_Handler->bit_order = PDM_FILTER_BIT_ORDER_LSB;
            BeamFormingInternal->Filter_M1_Delayed_Handler->endianness = PDM_FILTER_ENDIANNESS_LE;
            BeamFormingInternal->Filter_M1_Delayed_Handler->high_pass_tap = 1932735282;
            BeamFormingInternal->Filter_M1_Delayed_Handler->in_ptr_channels = 1;
            BeamFormingInternal->Filter_M1_Delayed_Handler->out_ptr_channels = 1;
            (void)PDM_Filter_Init(BeamFormingInternal->Filter_M1_Delayed_Handler);
            
            BeamFormingInternal->Filter_M1_Delayed_Config->output_samples_number = 16;
            BeamFormingInternal->Filter_M1_Delayed_Config->mic_gain = (int16_t)BeamFormingInternal->overall_gain;    
            BeamFormingInternal->Filter_M1_Delayed_Config->decimation_factor = dec_factor_temp;    
            (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)BeamFormingInternal->Filter_M1_Delayed_Handler, BeamFormingInternal->Filter_M1_Delayed_Config);
            
            BeamFormingInternal->Delay_M1->jump_in=BeamFormingInternal->ptr_M1_channels;
            BeamFormingInternal->Delay_M1->len=(uint16_t)(BeamFormingInternal->sampling_frequency)/8U;
            BeamFormingInternal->Delay_M1->delay=(uint16_t)roundf(((float32_t)BeamFormingInternal->mic_distance/10000.0f)/(SPEED_OF_SOUND/((float32_t)BeamFormingInternal->sampling_frequency*1000.0f)));
            Delay_Init(BeamFormingInternal->Delay_M1);
            
            /*TODO: optimize eventually*/
            BeamFormingInternal->Callbacks.PDM_Filter_Standard_M1=PDM_Filter;
            BeamFormingInternal->Callbacks.PDM_Filter_Standard_M2=PDM_Filter;
          }
          else
          {
            BeamFormingInternal->Delay_M1->jump_in=1;
            BeamFormingInternal->Delay_M1->len=16;
            BeamFormingInternal->Delay_M1->delay=1; 
            Delay_Init(BeamFormingInternal->Delay_M1);
          }        
        }    
      }
      else
      {
        /*PCM INPUT*/
        if (BeamFormingInternal->delay_enable == 1U)
        {
          BeamFormingInternal->Delay_M2->jump_in=1;
          BeamFormingInternal->Delay_M2->len=16;
          BeamFormingInternal->Delay_M2->delay=1;
          Delay_Init(BeamFormingInternal->Delay_M2);
          BeamFormingInternal->Callbacks.FirstStep=FirstStepInternal2;
          
          BeamFormingInternal->Delay_M1->jump_in=1;
          BeamFormingInternal->Delay_M1->len=16;
          BeamFormingInternal->Delay_M1->delay=1;  
          Delay_Init(BeamFormingInternal->Delay_M1);
          BeamFormingInternal->Callbacks.Delay=Delay_1;        
        }
        else
        {
          BeamFormingInternal->Callbacks.FirstStep=FirstStepInternal_NoDelay;
        }     
      }
      
      /************************************/
      float32_t internal_gain=1.0f;
      if(BeamFormingInternal->M2_gain>0.0f)
      {
        internal_gain=BeamFormingInternal->M2_gain;
      }
      if(BeamFormingInternal->mic_distance<=34U)
      {        
        BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_3mm[0]*256.0f),(coefficients_3mm[1]*256.0f),(coefficients_3mm[1]*256.0f*internal_gain));
      }
      else if(BeamFormingInternal->mic_distance<=45U)
      {        
        BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_4mm[0]*256.0f),(coefficients_4mm[1]*256.0f),(coefficients_4mm[1]*256.0f*internal_gain));
      }
      else if(BeamFormingInternal->mic_distance<=62U)
      {      
        BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_5_65mm[0]*256.0f),(coefficients_5_65mm[1]*256.0f),(coefficients_5_65mm[1]*256.0f*internal_gain)); 
      }
      else if(BeamFormingInternal->mic_distance<=80U)
      {      
        BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_7mm[0]*256.0f),(coefficients_7mm[1]*256.0f),(coefficients_7mm[1]*256.0f*internal_gain)); 
      }
      else if(BeamFormingInternal->mic_distance<=155U)
      {      
        BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_15mm[0]*256.0f),(coefficients_15mm[1]*256.0f),(coefficients_15mm[1]*256.0f*internal_gain));       
      }
      else if(BeamFormingInternal->mic_distance<=212U)    
      {
        BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_21_2mm[0]*256.0f),(coefficients_21_2mm[1]*256.0f),(coefficients_21_2mm[1]*256.0f*internal_gain));
      } 
      else
      {
        /* wrong distance: >212 is not supported*/
      } 
      
      if( (BeamFormingInternal->algorithm_type_init == ACOUSTIC_BF_TYPE_STRONG) || (BeamFormingInternal->algorithm_type_init == ACOUSTIC_BF_TYPE_ASR_READY) )
      {   
        float32_t gain=1.0f;
        if(BeamFormingInternal->M2_gain>0.0f)
        {
          gain=BeamFormingInternal->M2_gain;
        }
        if(BeamFormingInternal->mic_distance<=34U)
        {        
          BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_3mm[0]*256.0f),(coefficients_3mm[1]*256.0f*gain),(coefficients_3mm[1]*256.0f));
        }
        else if(BeamFormingInternal->mic_distance<=45U)
        {        
          BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_4mm[0]*256.0f),(coefficients_4mm[1]*256.0f*gain),(coefficients_4mm[1]*256.0f));
        }
        else if(BeamFormingInternal->mic_distance<=62U)
        {      
          BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_5_65mm[0]*256.0f),(coefficients_5_65mm[1]*256.0f*gain),(coefficients_5_65mm[1]*256.0f)); 
        }
        else if(BeamFormingInternal->mic_distance<=80U)
        {      
          BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_7mm[0]*256.0f),(coefficients_7mm[1]*256.0f*gain),(coefficients_7mm[1]*256.0f)); 
        }
        else if(BeamFormingInternal->mic_distance<=155U)
        {      
          BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_15mm[0]*256.0f),(coefficients_15mm[1]*256.0f*gain),(coefficients_15mm[1]*256.0f));       
        }
        else if(BeamFormingInternal->mic_distance<=212U)
        {
          BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_21_2mm[0]*256.0f),(coefficients_21_2mm[1]*256.0f*gain),(coefficients_21_2mm[1]*256.0f));
        }        
        else
        {
          /* wrong distance: >212 is not supported*/
        }
      }
    }
    /*****************/
  }
  else
  {
    ret |= ACOUSTIC_BF_TYPE_ERROR;    
  }
  return ret;
}


static uint32_t libBeamforming_getConfig(AcousticBF_Handler_t * pHandler, AcousticBF_Config_t * pConfig)
{  
  libBeamforming_Handler_Internal * BeamFormingInternal = (libBeamforming_Handler_Internal *)(pHandler->pInternalMemory);
  
  pConfig->algorithm_type=BeamFormingInternal->algorithm_type_init;
  if(BeamFormingInternal->M2_gain==0.0f)
  {
    pConfig->M2_gain=BeamFormingInternal->gain_0;
  }else
  {
    pConfig->M2_gain=BeamFormingInternal->M2_gain;
  }
  
  pConfig->volume=(int8_t)(BeamFormingInternal->overall_gain);
  pConfig->mic_distance=BeamFormingInternal->mic_distance;
  pConfig->mic_distance=BeamFormingInternal->mic_distance;
  return 0;  
}

static uint32_t libBeamforming_getMemorySize(AcousticBF_Handler_t * pHandler)
{
  volatile uint32_t  byte_offset = sizeof(libBeamforming_Handler_Internal);
  byte_offset+=INTERNAL_BUFF_SIZE*sizeof(int16_t);  //DIR1_BUFF 
  byte_offset+=INTERNAL_BUFF_SIZE*sizeof(int16_t); //DIR2_BUFF 
  byte_offset+=INTERNAL_BUFF_SIZE*sizeof(int16_t); //E_BUFF  
  byte_offset+=sizeof(PDM_Filter_Handler_t);      //Filter_M1_Standard_Handler
  byte_offset+=sizeof(PDM_Filter_Handler_t);   //Filter_M2_Delayed_Handler
  byte_offset+=sizeof(TDelay);   //TDelay_M2
  byte_offset+=sizeof(PDM_Filter_Handler_t);   //Filter_M2_Standard_Handler
  byte_offset+=sizeof(TPCMBF);     //BFParam_1
  byte_offset+=sizeof(TPCMBF);   //BFParam_2
  byte_offset+=sizeof(PDM_Filter_Handler_t);    //Filter_M1_Delayed_Handler
  byte_offset+=sizeof(TDelay);     //Delay_M1  
  byte_offset+=4U*sizeof(PDM_Filter_Config_t);      //Filter_Configd_Handler
  
#ifndef STANDARD_IIR
  byte_offset+=sizeof(arm_biquad_casd_df1_inst_q15);   //S1
  byte_offset+=sizeof(arm_biquad_casd_df1_inst_q15);   //S2
#endif    
  /********** DENOISER (FOR LIGHT OR STRONG VERSIONS)********/
  if((pHandler->algorithm_type_init == ACOUSTIC_BF_TYPE_CARDIOID_DENOISE) || (pHandler->algorithm_type_init == ACOUSTIC_BF_TYPE_STRONG))
  { 
    byte_offset+=sizeof(SpeexPreprocessState); 
    byte_offset+=sizeof(drft_lookup); 
    byte_offset+=sizeof(FilterBank);    
  }  
  /********** ADAPTIVE (FOR ASR OR STRONG VERSIONS)********/
  if((pHandler->algorithm_type_init == ACOUSTIC_BF_TYPE_ASR_READY) || (pHandler->algorithm_type_init == ACOUSTIC_BF_TYPE_STRONG))
  {    
    byte_offset+=sizeof(SpeexEchoState);    
    byte_offset+=sizeof(drft_lookup); 
  }   
  while( (++byte_offset%4U)!=0U )
  {    
  }
  pHandler->internal_memory_size= byte_offset;
  return 0;
}

static uint8_t FirstStepInternal1(void *pM1, void *pM2, int16_t *ptr_Out, libBeamforming_Handler_Internal * BeamFormingInternal)
{
  uint8_t ret = 0;
  uint16_t i;  
  uint32_t GainSamplesCounter_0 = BeamFormingInternal->GainSamplesCounter_0;  
  float32_t RMSm1_0 =BeamFormingInternal->RMSm1_0;
  float32_t RMSm2_0 = BeamFormingInternal->RMSm2_0;
  float32_t gain_0 = BeamFormingInternal->gain_old_0;
  float32_t gain_old_0 = BeamFormingInternal->gain_old_0;  
  // CARDIOIDE 1  
  BeamFormingInternal->Callbacks.Delay(pM2,BeamFormingInternal,BeamFormingInternal->Delay_M2); 
  
  BeamFormingInternal->Callbacks.PDM_Filter_Standard_M1((uint8_t*)(pM1), (uint16_t*)BeamFormingInternal->OutBuff, BeamFormingInternal->Filter_M1_Standard_Handler);
  BeamFormingInternal->Callbacks.PDM_Filter_Delayed((uint8_t*)(BeamFormingInternal->PDM_Delay_Buffer), (uint16_t*)&(BeamFormingInternal->OutBuff[16]), BeamFormingInternal->Filter_M2_Delayed_Handler);  
  if(BeamFormingInternal->M2_gain==0.0f)
  {    
    for( i = 0; i<16U;i++)
    {
      RMSm1_0+=((float32_t)((int16_t)BeamFormingInternal->OutBuff[i])*(float32_t)((int16_t)BeamFormingInternal->OutBuff[i]));
      RMSm2_0+=((float32_t)((int16_t)BeamFormingInternal->OutBuff[i+16U])*(float32_t)((int16_t)BeamFormingInternal->OutBuff[i+16U]));
      GainSamplesCounter_0++;    
    }    
    if(GainSamplesCounter_0==GAIN_COMPUTATION_LENGTH)
    {
      if(RMSm1_0 < 300000000.0f)
      {
        float32_t RMSgain = RMSm1_0/RMSm2_0;
        if (RMSgain >= 0.0f)
        {
          gain_0=((sqrtf(RMSgain))*0.2f)+(gain_old_0*0.8f);
        }
        if(gain_0 < 1.0f)
        {
          gain_0 = 1.0f;
        }
        BeamFormingInternal->gain_0 = gain_0;        
#ifdef STANDARD_IIR        
        BeamFormer_SetGain( BeamFormingInternal);        
#endif        
        gain_old_0=gain_0;        
      }      
      GainSamplesCounter_0=0;
      RMSm1_0=0.0f;
      RMSm2_0=0.0f;      
    }
  }  
#ifdef STANDARD_IIR
  BeamFormer( (pcm_sample*) BeamFormingInternal->OutBuff, (pcm_sample*) &(BeamFormingInternal->OutBuff[16]), BeamFormingInternal->OutBuff_Beam, 16,(TPCMBF *)BeamFormingInternal->BFParam_1, BeamFormingInternal);    
#else
  BeamFormer_1( (pcm_sample*) BeamFormingInternal->OutBuff, (pcm_sample*) &(BeamFormingInternal->OutBuff[16]), BeamFormingInternal->OutBuff_Beam, 16,(TPCMBF *)BeamFormingInternal->BFParam_1, BeamFormingInternal);     
#endif    
  if((BeamFormingInternal->algorithm_type_init==ACOUSTIC_BF_TYPE_STRONG) || (BeamFormingInternal->algorithm_type_init==ACOUSTIC_BF_TYPE_ASR_READY) || (BeamFormingInternal->ref_mic_enable == 7U) )
  {
    // CARDIOIDE 2 
    BeamFormingInternal->Callbacks.Delay(pM1,BeamFormingInternal,BeamFormingInternal->Delay_M1); 
    BeamFormingInternal->Callbacks.PDM_Filter_Standard_M2((uint8_t*)(pM2),  &(BeamFormingInternal->OutBuff)[32], BeamFormingInternal->Filter_M2_Standard_Handler);    
    BeamFormingInternal->Callbacks.PDM_Filter_Delayed((uint8_t*)(BeamFormingInternal->PDM_Delay_Buffer), &(BeamFormingInternal->OutBuff[48]), BeamFormingInternal->Filter_M1_Delayed_Handler);    
#ifdef STANDARD_IIR
    BeamFormer( (pcm_sample*) &(BeamFormingInternal->OutBuff)[32], (pcm_sample*) &(BeamFormingInternal->OutBuff[48]), &(BeamFormingInternal->OutBuff_Beam)[16], 16,(TPCMBF *)BeamFormingInternal->BFParam_2, BeamFormingInternal);     
#else
    BeamFormer_2( (pcm_sample*) &(BeamFormingInternal->OutBuff)[32], (pcm_sample*) &(BeamFormingInternal->OutBuff[48]), &(BeamFormingInternal->OutBuff_Beam)[16], 16,(TPCMBF *)BeamFormingInternal->BFParam_2, BeamFormingInternal);    
#endif    
  }  
  uint32_t offset=BeamFormingInternal->ptr_out_channels;
  for(i=0; i < 16U; i++)
  {
    /* Input */
    BeamFormingInternal->Dir1_Buffer[BeamFormingInternal->Samples_Count] =(int16_t)(BeamFormingInternal->OutBuff_Beam[i]);
    BeamFormingInternal->Dir2_Buffer[BeamFormingInternal->Samples_Count] =(int16_t)(BeamFormingInternal->OutBuff_Beam[16U+i]);
    BeamFormingInternal->Samples_Count++;
    if(BeamFormingInternal->Samples_Count==ECHO_BUFF)
    {
      BeamFormingInternal->Buffer_State=1;
    }
    else if(BeamFormingInternal->Samples_Count==(ECHO_BUFF*2U))
    {
      BeamFormingInternal->Buffer_State=2;
      BeamFormingInternal->Samples_Count=0;
    }
    else
    {
      /* other values for Sampels Count are not supported */
    }
    /* Output */         
    ptr_Out[(i*offset)] = BeamFormingInternal->E_Buffer[BeamFormingInternal->Samples_Count_Output];    
    if(BeamFormingInternal->ref_mic_enable == 1U)    
    {
      ptr_Out[(i*offset)+1U] = (int16_t)(BeamFormingInternal->OutBuff[i]);
    }    
    if((BeamFormingInternal->ref_mic_enable == 7U))
    {
      ptr_Out[(i*offset)+1U] = (int16_t)(BeamFormingInternal->OutBuff_Beam[16U+i]);
    }   
    
    BeamFormingInternal->Samples_Count_Output++;
    if(BeamFormingInternal->Samples_Count_Output==(ECHO_BUFF*2U))
    {
      BeamFormingInternal->Samples_Count_Output = 0;
    }
  }  
  BeamFormingInternal->GainSamplesCounter_0=GainSamplesCounter_0;   
  BeamFormingInternal->RMSm1_0 = RMSm1_0;
  BeamFormingInternal->RMSm2_0 = RMSm2_0;
  BeamFormingInternal->gain_old_0 = gain_old_0;    
  BeamFormingInternal->counter++;    
  if(BeamFormingInternal->counter==8U)
  {
    BeamFormingInternal->counter=0;
    ret = 1; 
  }    
  
  return ret;  
}

static uint8_t FirstStepInternal_NoDelay(void *pM1, void *pM2, int16_t *ptr_Out, libBeamforming_Handler_Internal * BeamFormingInternal)
{
  uint8_t ret = 0;
  uint16_t i;  
  uint32_t GainSamplesCounter_0 = BeamFormingInternal->GainSamplesCounter_0;  
  float32_t RMSm1_0 =BeamFormingInternal->RMSm1_0;
  float32_t RMSm2_0 = BeamFormingInternal->RMSm2_0;
  float32_t gain_0 = BeamFormingInternal->gain_0;  
  float32_t gain_old_0 = BeamFormingInternal->gain_old_0;  
  // CARDIOIDE 1  
  // CARDIOIDE 1  
  for(i=0;i<16U;i++)
  {
    //OutBuff: M1 M2 M1d M2d
    BeamFormingInternal->OutBuff[i]=((uint16_t *)(pM1))[i];
    BeamFormingInternal->OutBuff[16U+i]=((uint16_t *)(pM2))[i];
    BeamFormingInternal->OutBuff[32U+i]=((uint16_t *)(pM1))[i+16U];
    BeamFormingInternal->OutBuff[48U+i]=((uint16_t *)(pM2))[i+16U];
  } 
  
  if(BeamFormingInternal->M2_gain==0.0f)
  {
    for( i = 0; i<16U;i++)
    {
      RMSm1_0+=((float32_t)(((int16_t *)(BeamFormingInternal->OutBuff))[i])*(float32_t)(((int16_t *)(BeamFormingInternal->OutBuff))[i]));
      RMSm2_0+=((float32_t)((int16_t)BeamFormingInternal->OutBuff[48U+i])*(float32_t)((int16_t)BeamFormingInternal->OutBuff[48U+i]));  
      GainSamplesCounter_0++;    
    }    
    if(GainSamplesCounter_0==GAIN_COMPUTATION_LENGTH)
    {
      if(RMSm1_0 < 300000000.0f)
      {
        float32_t RMSgain = RMSm1_0/RMSm2_0;
        if (RMSgain >= 0.0f)
      {
          gain_0=((sqrtf(RMSgain))*0.2f)+(gain_old_0*0.8f);
        } 
        if(gain_0 < 1.0f)
        {
          gain_0 = 1.0f;
        }
        BeamFormingInternal->gain_0 = gain_0;
#ifdef STANDARD_IIR        
        BeamFormer_SetGain( BeamFormingInternal);        
#endif        
        gain_old_0=gain_0;        
      }      
      GainSamplesCounter_0=0;
      RMSm1_0=0.0f;
      RMSm2_0=0.0f;
    }
  }
  
#ifdef STANDARD_IIR
  BeamFormer( (pcm_sample*)&BeamFormingInternal->OutBuff[0], (pcm_sample*)&BeamFormingInternal->OutBuff[48], BeamFormingInternal->OutBuff_Beam, 16,(TPCMBF *)BeamFormingInternal->BFParam_1, BeamFormingInternal);     
#else
  BeamFormer_1( (pcm_sample*)&BeamFormingInternal->OutBuff[0], (pcm_sample*)&BeamFormingInternal->OutBuff[48], BeamFormingInternal->OutBuff_Beam, 16,(TPCMBF *)BeamFormingInternal->BFParam_1, BeamFormingInternal); 
#endif  
  if((BeamFormingInternal->algorithm_type_init==ACOUSTIC_BF_TYPE_STRONG) || (BeamFormingInternal->algorithm_type_init==ACOUSTIC_BF_TYPE_ASR_READY) || (BeamFormingInternal->ref_mic_enable == 7U))
  {
#ifdef STANDARD_IIR
    BeamFormer( (pcm_sample*) &BeamFormingInternal->OutBuff[16], (pcm_sample*)&BeamFormingInternal->OutBuff[32], &(BeamFormingInternal->OutBuff_Beam)[16], 16,(TPCMBF *)BeamFormingInternal->BFParam_2, BeamFormingInternal);    
#else
    BeamFormer_2( (pcm_sample*) &BeamFormingInternal->OutBuff[16], (pcm_sample*) &BeamFormingInternal->OutBuff[32], &(BeamFormingInternal->OutBuff_Beam)[16], 16,(TPCMBF *)BeamFormingInternal->BFParam_2, BeamFormingInternal);    
#endif    
  }  
  uint32_t offset=BeamFormingInternal->ptr_out_channels;
  
  for(i=0; i < 16U; i++)
  {
    /* Input */
    BeamFormingInternal->Dir1_Buffer[BeamFormingInternal->Samples_Count] = (int16_t)(BeamFormingInternal->OutBuff_Beam[i]);
    BeamFormingInternal->Dir2_Buffer[BeamFormingInternal->Samples_Count] = (int16_t)(BeamFormingInternal->OutBuff_Beam[16U+i]);
    BeamFormingInternal->Samples_Count++;
    if(BeamFormingInternal->Samples_Count==ECHO_BUFF)
    {
      BeamFormingInternal->Buffer_State=1;
    }
    else if(BeamFormingInternal->Samples_Count==(ECHO_BUFF*2U))
    {
      BeamFormingInternal->Buffer_State=2;
      BeamFormingInternal->Samples_Count=0;
    }
  else
  {
      /* other values for Sampels Count are not supported */
    }
    /* Output */         
    ptr_Out[(i*offset)] = BeamFormingInternal->E_Buffer[BeamFormingInternal->Samples_Count_Output];    
    if(BeamFormingInternal->ref_mic_enable == 1U)    
    {
      ptr_Out[(i*offset)+1U] = (int16_t)(BeamFormingInternal->OutBuff[i]);
    }    
    if((BeamFormingInternal->ref_mic_enable == 7U))
    {
      ptr_Out[(i*offset)+1U] = (int16_t)(BeamFormingInternal->OutBuff_Beam[16U+i]);
    }
    
    BeamFormingInternal->Samples_Count_Output++;
    if(BeamFormingInternal->Samples_Count_Output==(ECHO_BUFF*2U))
    {
      BeamFormingInternal->Samples_Count_Output = 0;
    }
  }  
  BeamFormingInternal->GainSamplesCounter_0=GainSamplesCounter_0;   
  BeamFormingInternal->RMSm1_0 = RMSm1_0;
  BeamFormingInternal->RMSm2_0 = RMSm2_0;
  BeamFormingInternal->gain_old_0 = gain_old_0;    
  BeamFormingInternal->counter++;    
  if(BeamFormingInternal->counter==8U)
  {
    BeamFormingInternal->counter=0;
    ret = 1; 
}

  return ret;
}

static uint8_t FirstStepInternal3(void *pM1, void *pM2, int16_t *ptr_Out, libBeamforming_Handler_Internal * BeamFormingInternal)
{  
  uint8_t ret = 0;
  uint16_t i;  
  uint32_t GainSamplesCounter_0 = BeamFormingInternal->GainSamplesCounter_0;  
  float32_t RMSm1_0 =BeamFormingInternal->RMSm1_0;
  float32_t RMSm2_0 = BeamFormingInternal->RMSm2_0;
  float32_t gain_0 = BeamFormingInternal->gain_0; 
  float32_t gain_old_0 = BeamFormingInternal->gain_old_0;  
  // CARDIOIDE 1  
  BeamFormingInternal->Callbacks.PDM_Filter_Standard_M1((uint8_t*)(pM1), BeamFormingInternal->OutBuff, BeamFormingInternal->Filter_M1_Standard_Handler);
  BeamFormingInternal->Callbacks.PDM_Filter_Standard_M2((uint8_t*)(pM2), &(BeamFormingInternal->OutBuff[16]), BeamFormingInternal->Filter_M2_Standard_Handler);  
  for(i=0;i<16U;i++)
  {
    BeamFormingInternal->OutBuff[i+48U]=BeamFormingInternal->OutBuff[16U+i];
  }  
  BeamFormingInternal->Callbacks.Delay( &BeamFormingInternal->OutBuff[48],BeamFormingInternal,BeamFormingInternal->Delay_M2);  
  if(BeamFormingInternal->M2_gain==0.0f)
  {
    for( i = 0; i<16U;i++)
    {
      RMSm1_0+=((float32_t)((int16_t)BeamFormingInternal->OutBuff[i])*(float32_t)((int16_t)BeamFormingInternal->OutBuff[i]));
      RMSm2_0+=((float32_t)((int16_t)BeamFormingInternal->OutBuff[i+48U])*(float32_t)((int16_t)BeamFormingInternal->OutBuff[i+48U]));  //check if ok, otherwise delayed
      GainSamplesCounter_0++;    
    }    
    if(GainSamplesCounter_0==GAIN_COMPUTATION_LENGTH)
    {
      if(RMSm1_0 < 300000000.0f)
      {
        float32_t RMSgain = RMSm1_0/RMSm2_0;
        if (RMSgain >= 0.0f)
      {
          gain_0=((sqrtf(RMSgain))*0.2f)+(gain_old_0*0.8f);
        }  
        if(gain_0 < 1.0f)
        {
          gain_0 = 1.0f;
        }
        BeamFormingInternal->gain_0 = gain_0;
#ifdef STANDARD_IIR        
        BeamFormer_SetGain( BeamFormingInternal);        
#endif       
        gain_old_0=gain_0;        
      }      
      GainSamplesCounter_0=0;
      RMSm1_0=0.0f;
      RMSm2_0=0.0f;
    }
  }  
#ifdef STANDARD_IIR
  BeamFormer( (pcm_sample*) BeamFormingInternal->OutBuff, (pcm_sample*) &BeamFormingInternal->OutBuff[48], BeamFormingInternal->OutBuff_Beam, 16,(TPCMBF *)BeamFormingInternal->BFParam_1, BeamFormingInternal);     
#else
  BeamFormer_1( (pcm_sample*) BeamFormingInternal->OutBuff, (pcm_sample*) &BeamFormingInternal->OutBuff[48], BeamFormingInternal->OutBuff_Beam, 16,(TPCMBF *)BeamFormingInternal->BFParam_1, BeamFormingInternal);   
#endif  
  if((BeamFormingInternal->algorithm_type_init==ACOUSTIC_BF_TYPE_STRONG) || (BeamFormingInternal->algorithm_type_init==ACOUSTIC_BF_TYPE_ASR_READY) || (BeamFormingInternal->ref_mic_enable == 7U)){    
    for(i=0;i<16U;i++)
    {
      BeamFormingInternal->OutBuff[i+32U]=BeamFormingInternal->OutBuff[i];
    }
    // CARDIOIDE 2 
    BeamFormingInternal->Callbacks.Delay(&BeamFormingInternal->OutBuff[32],BeamFormingInternal,BeamFormingInternal->Delay_M1);    
#ifdef STANDARD_IIR
    BeamFormer( (pcm_sample*) &(BeamFormingInternal->OutBuff)[16], (pcm_sample*)&BeamFormingInternal->OutBuff[32], &(BeamFormingInternal->OutBuff_Beam)[16], 16,(TPCMBF *)BeamFormingInternal->BFParam_2, BeamFormingInternal);     
#else
    BeamFormer_2( (pcm_sample*) &(BeamFormingInternal->OutBuff)[16], (pcm_sample*)&BeamFormingInternal->OutBuff[32], &(BeamFormingInternal->OutBuff_Beam)[16], 16,(TPCMBF *)BeamFormingInternal->BFParam_2, BeamFormingInternal);    
#endif    
  }  
  uint32_t offset=BeamFormingInternal->ptr_out_channels;
  for(i=0; i < 16U; i++)
  {
    /* Input */
    BeamFormingInternal->Dir1_Buffer[BeamFormingInternal->Samples_Count] = (int16_t)(BeamFormingInternal->OutBuff_Beam[i]);
    BeamFormingInternal->Dir2_Buffer[BeamFormingInternal->Samples_Count] = (int16_t)(BeamFormingInternal->OutBuff_Beam[16U+i]);
    BeamFormingInternal->Samples_Count++;
    if(BeamFormingInternal->Samples_Count==ECHO_BUFF)
    {
      BeamFormingInternal->Buffer_State=1;
    }
    else if(BeamFormingInternal->Samples_Count==(ECHO_BUFF*2U))
    {
      BeamFormingInternal->Buffer_State=2;
      BeamFormingInternal->Samples_Count=0;
    }
    else
    {
      /* other values for Sampels Count are not supported */
    }
    /* Output */     
    ptr_Out[(i*offset)] = BeamFormingInternal->E_Buffer[BeamFormingInternal->Samples_Count_Output];    
    if(BeamFormingInternal->ref_mic_enable ==1U)
    {
      ptr_Out[(i*offset)+1U] = (int16_t)(BeamFormingInternal->OutBuff[i]);
    }    
    if((BeamFormingInternal->ref_mic_enable == 7U))
    {
      ptr_Out[(i*offset)+1U] = (int16_t)(BeamFormingInternal->OutBuff_Beam[16U+i]);
    }
    BeamFormingInternal->Samples_Count_Output++;
    if(BeamFormingInternal->Samples_Count_Output==(ECHO_BUFF*2U))
    {
      BeamFormingInternal->Samples_Count_Output = 0;    
    }
  }  
  BeamFormingInternal->GainSamplesCounter_0=GainSamplesCounter_0;    
  BeamFormingInternal->RMSm1_0 = RMSm1_0;
  BeamFormingInternal->RMSm2_0 = RMSm2_0;
  BeamFormingInternal->gain_old_0 = gain_old_0;    
  BeamFormingInternal->counter++;    
  if(BeamFormingInternal->counter==8U)
  {
    BeamFormingInternal->counter=0;
    ret = 1; 
}

  return ret;
  }
  
static uint8_t FirstStepInternal2(void *pM1, void *pM2, int16_t *ptr_Out, libBeamforming_Handler_Internal * BeamFormingInternal)
  {
  uint8_t ret = 0;
  uint16_t i;  
  uint32_t GainSamplesCounter_0 = BeamFormingInternal->GainSamplesCounter_0;  
  float32_t RMSm1_0 =BeamFormingInternal->RMSm1_0;
  float32_t RMSm2_0 = BeamFormingInternal->RMSm2_0;
  float32_t gain_0 = BeamFormingInternal->gain_0; 
  float32_t gain_old_0 = BeamFormingInternal->gain_old_0;  
  // CARDIOIDE 1  
  for(i=0;i<16U;i++)
  {
    //OutBuff: M1 M2 M1d M2d
    BeamFormingInternal->OutBuff[i]=((uint16_t *)(pM1))[i*BeamFormingInternal->ptr_M1_channels];
    BeamFormingInternal->OutBuff[16U+i]=((uint16_t *)(pM2))[i*BeamFormingInternal->ptr_M2_channels];
    BeamFormingInternal->OutBuff[48U+i]=((uint16_t *)(pM2))[i*BeamFormingInternal->ptr_M2_channels];/**BeamFormingInternal->OutBuff[16+i];**/
  }  
  BeamFormingInternal->Callbacks.Delay(&BeamFormingInternal->OutBuff[48],BeamFormingInternal,BeamFormingInternal->Delay_M2);  
  if(BeamFormingInternal->M2_gain==0.0f)
  {
    for( i = 0; i<16U;i++)
    {
      RMSm1_0+=((float32_t)(((int16_t *)(BeamFormingInternal->OutBuff))[i])*(float32_t)(((int16_t *)(BeamFormingInternal->OutBuff))[i]));
      RMSm2_0+=((float32_t)((int16_t)BeamFormingInternal->OutBuff[48U+i])*(float32_t)((int16_t)BeamFormingInternal->OutBuff[48U+i]));  
      GainSamplesCounter_0++;    
    }    
    if(GainSamplesCounter_0==GAIN_COMPUTATION_LENGTH)
    {
      if(RMSm1_0 < 300000000.0f)
      {
        float32_t RMSgain = RMSm1_0/RMSm2_0;
        if (RMSgain >= 0.0f)
        {
          gain_0=((sqrtf(RMSgain))*0.2f)+(gain_old_0*0.8f);
        } 
        if(gain_0 < 1.0f)
        {
          gain_0 = 1.0f;
        }
        BeamFormingInternal->gain_0 = gain_0;
#ifdef STANDARD_IIR        
        BeamFormer_SetGain( BeamFormingInternal);        
#endif        
        gain_old_0=gain_0;        
      }      
      GainSamplesCounter_0=0;
      RMSm1_0=0.0f;
      RMSm2_0=0.0f;
    }
  }  
#ifdef STANDARD_IIR
  BeamFormer( (pcm_sample*)&BeamFormingInternal->OutBuff[0], (pcm_sample*)&BeamFormingInternal->OutBuff[48], BeamFormingInternal->OutBuff_Beam, 16,(TPCMBF *)BeamFormingInternal->BFParam_1, BeamFormingInternal);     
#else
  BeamFormer_1( (pcm_sample*)&BeamFormingInternal->OutBuff[0], (pcm_sample*)&BeamFormingInternal->OutBuff[48], BeamFormingInternal->OutBuff_Beam, 16,(TPCMBF *)BeamFormingInternal->BFParam_1, BeamFormingInternal); 
#endif  
  if((BeamFormingInternal->algorithm_type_init==ACOUSTIC_BF_TYPE_STRONG) || (BeamFormingInternal->algorithm_type_init==ACOUSTIC_BF_TYPE_ASR_READY) || (BeamFormingInternal->ref_mic_enable == 7U)){ 
    for(i=0;i<16U;i++)
    {
      BeamFormingInternal->OutBuff[32U+i]=((uint16_t *)(pM1))[i*BeamFormingInternal->ptr_M1_channels];
    }
    // CARDIOIDE 2 
    BeamFormingInternal->Callbacks.Delay(&BeamFormingInternal->OutBuff[32],BeamFormingInternal,BeamFormingInternal->Delay_M1);    
#ifdef STANDARD_IIR
    BeamFormer( (pcm_sample*) &BeamFormingInternal->OutBuff[16], (pcm_sample*)&BeamFormingInternal->OutBuff[32], &(BeamFormingInternal->OutBuff_Beam)[16], 16,(TPCMBF *)BeamFormingInternal->BFParam_2, BeamFormingInternal);    
#else
    BeamFormer_2( (pcm_sample*) &BeamFormingInternal->OutBuff[16], (pcm_sample*) &BeamFormingInternal->OutBuff[32], &(BeamFormingInternal->OutBuff_Beam)[16], 16,(TPCMBF *)BeamFormingInternal->BFParam_2, BeamFormingInternal);    
#endif    
  }  
  uint32_t offset=BeamFormingInternal->ptr_out_channels;
  for(i=0; i < 16U; i++)
  {
    /* Input */
    BeamFormingInternal->Dir1_Buffer[BeamFormingInternal->Samples_Count] = (int16_t)(BeamFormingInternal->OutBuff_Beam[i]);
    BeamFormingInternal->Dir2_Buffer[BeamFormingInternal->Samples_Count] = (int16_t)(BeamFormingInternal->OutBuff_Beam[16U+i]);
    BeamFormingInternal->Samples_Count++;
    if(BeamFormingInternal->Samples_Count==ECHO_BUFF)
    {
      BeamFormingInternal->Buffer_State=1;
    }
    else if(BeamFormingInternal->Samples_Count==(ECHO_BUFF*2U))
    {
      BeamFormingInternal->Buffer_State=2;
      BeamFormingInternal->Samples_Count=0;
    }
    else
    {
      /* other values for Sampels Count are not supported */
    }
    /* Output */    
    ptr_Out[(i*offset)] = BeamFormingInternal->E_Buffer[BeamFormingInternal->Samples_Count_Output];    
    if(BeamFormingInternal->ref_mic_enable == 1U)
    {
      ptr_Out[(i*offset)+1U] = (int16_t)(BeamFormingInternal->OutBuff[i]);
    }    
    if((BeamFormingInternal->ref_mic_enable == 7U))
    {
      ptr_Out[(i*offset)+1U] = (int16_t)(BeamFormingInternal->OutBuff_Beam[16U+i]);
    }    
    BeamFormingInternal->Samples_Count_Output++;
    if(BeamFormingInternal->Samples_Count_Output==(ECHO_BUFF*2U))
    {
      BeamFormingInternal->Samples_Count_Output = 0;
    }
  }   
  BeamFormingInternal->GainSamplesCounter_0=GainSamplesCounter_0;  
  BeamFormingInternal->RMSm1_0 = RMSm1_0;
  BeamFormingInternal->RMSm2_0 = RMSm2_0;
  BeamFormingInternal->gain_old_0 = gain_old_0;    
  BeamFormingInternal->counter++;    
  if(BeamFormingInternal->counter==8U)
  {
    BeamFormingInternal->counter=0;
    ret = 1; 
  }    
  
  return ret;
}

static void BeamFormer_SetGain(libBeamforming_Handler_Internal * BeamFormingInternal)
{
  float32_t gain = BeamFormingInternal->gain_0;
  
  if(BeamFormingInternal->mic_distance<=34U)
  {        
    BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_3mm[0]*256.0f),(coefficients_3mm[1]*256.0f),(coefficients_3mm[1]*256.0f*gain));
  }
  else if(BeamFormingInternal->mic_distance<=40U)
  {      
    BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_4mm[0]*256.0f),(coefficients_4mm[1]*256.0f),(coefficients_4mm[1]*256.0f*gain));
  }
  else if(BeamFormingInternal->mic_distance<=57U)
  {      
    BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_5_65mm[0]*256.0f),(coefficients_5_65mm[1]*256.0f),(coefficients_5_65mm[1]*256.0f*gain)); 
  }
  else if(BeamFormingInternal->mic_distance<=80U)
  {      
    BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_7mm[0]*256.0f),(coefficients_7mm[1]*256.0f),(coefficients_7mm[1]*256.0f*gain)); 
  }
  else if(BeamFormingInternal->mic_distance<=150U)
  {
    BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_15mm[0]*256.0f),(coefficients_15mm[1]*256.0f),(coefficients_15mm[1]*256.0f*gain));       
  }
  else if(BeamFormingInternal->mic_distance<=212U)
  {
    BF_Init(BeamFormingInternal->BFParam_1, 16000, 1, (coefficients_21_2mm[0]*256.0f),(coefficients_21_2mm[1]*256.0f),(coefficients_21_2mm[1]*256.0f*gain));
  }
  else
  {
    /* wrong distance: >212 is not supported*/
  }
  
  if((BeamFormingInternal->initialized_as == ACOUSTIC_BF_TYPE_STRONG) || (BeamFormingInternal->initialized_as == ACOUSTIC_BF_TYPE_ASR_READY))
  {
    
    if(BeamFormingInternal->mic_distance<=34U)
    {        
      BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_3mm[0]*256.0f),(coefficients_3mm[1]*256.0f*gain),(coefficients_3mm[1]*256.0f));
    }
    else if(BeamFormingInternal->mic_distance<=40U)
    {        
      BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_4mm[0]*256.0f),(coefficients_4mm[1]*256.0f*gain),(coefficients_4mm[1]*256.0f));
    }
    else if(BeamFormingInternal->mic_distance<=57U)
    {      
      BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_5_65mm[0]*256.0f),(coefficients_5_65mm[1]*256.0f*gain),(coefficients_5_65mm[1]*256.0f)); 
    }
    else if(BeamFormingInternal->mic_distance<=80U)
    {      
      BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_7mm[0]*256.0f),(coefficients_7mm[1]*256.0f*gain),(coefficients_7mm[1]*256.0f)); 
    }
    else if(BeamFormingInternal->mic_distance<=150U)
    {      
      BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_15mm[0]*256.0f),(coefficients_15mm[1]*256.0f*gain),(coefficients_15mm[1]*256.0f));       
    }
    else if(BeamFormingInternal->mic_distance<=212U)
    {      
      BF_Init(BeamFormingInternal->BFParam_2, 16000, 1, (coefficients_21_2mm[0]*256.0f),(coefficients_21_2mm[1]*256.0f*gain),(coefficients_21_2mm[1]*256.0f));
    }
    else
    {
      /* wrong distance: >212 is not supported*/
    } 
  }  
}

static void Delay_Init(TDelay *Delay_struct) 
{  
  /*TODO FOR*/
  (Delay_struct->last_part_64)= 0;
  Delay_struct->nBytes = (uint8_t)(Delay_struct->delay/8U);
  Delay_struct->nBits = (uint8_t)(Delay_struct->delay%8U);
}

static void Delay_1(void * buffer_in, libBeamforming_Handler_Internal * BeamFormingInternal ,TDelay *Delay_struct)
{
  UNUSED(BeamFormingInternal);
  uint16_t i;  
  uint16_t app1 = ((uint16_t *)(buffer_in))[(Delay_struct->len-1U)* Delay_struct->jump_in];  
  for(i=15U;i>0U;i--)
  {
    ((uint16_t *)(buffer_in))[Delay_struct->jump_in*i]=((uint16_t *)(buffer_in))[Delay_struct->jump_in*(i-1U)];      
  }  
  ((uint16_t *)(buffer_in))[0] = (uint16_t)(*((uint16_t *)(Delay_struct->last_part)));
  *((uint16_t *)(Delay_struct->last_part)) = app1;  
}

//pdmDelay up to 64 samples
static void Delay_2(void * buffer_in, libBeamforming_Handler_Internal * BeamFormingInternal ,TDelay *Delay_struct) 
{
  uint16_t i;	
  uint64_t app,app2,app1;
  uint8_t * buffer_out = BeamFormingInternal->PDM_Delay_Buffer;
  uint16_t jump_in = Delay_struct->jump_in;
  uint16_t delay = Delay_struct->delay;
  app1 = Delay_struct->last_part_64;
  uint16_t nBytesTotal = (uint16_t)BeamFormingInternal->sampling_frequency/64U;
  
  for(i=0;i<nBytesTotal;i++)
  {
    app =(((uint8_t *)buffer_in)[((i*8U))*jump_in] << 24) +(((uint8_t *)buffer_in)[((i*8U)+1U)*jump_in] << 16) +(((uint8_t *)buffer_in)[((i*8U)+2U)*jump_in] << 8) +(((uint8_t *)buffer_in)[((i*8U)+3U)*jump_in] );
    uint32_t app01 =  (((uint8_t *)buffer_in)[((i*8U)+4U)*jump_in] << 24) + (((uint8_t *)buffer_in)[((i*8U)+5U)*jump_in] << 16) + (((uint8_t *)buffer_in)[((i*8U)+6U)*jump_in] << 8) + ((uint8_t *)buffer_in)[((i*8U)+7U)*jump_in];
    app=(app << 32) | app01; 
    app2 = app>>delay;
    app2 = app2 | app1;
    buffer_out[7] = (uint8_t)(app2&0xffu);
    app2=app2>>8;
    buffer_out[6] = (uint8_t)(app2&0xffu);
    app2=app2>>8;
    buffer_out[5] = (uint8_t)(app2&0xffu);
    app2=app2>>8;
    buffer_out[4] = (uint8_t)(app2&0xffu); 
    app2=app2>>8;
    buffer_out[3] = (uint8_t)(app2&0xffu);
    app2=app2>>8;
    buffer_out[2] = (uint8_t)(app2&0xffu);
    app2=app2>>8;
    buffer_out[1] = (uint8_t)(app2&0xffu);
    app2=app2>>8;
    buffer_out[0] = (uint8_t)(app2&0xffu);
    buffer_out+=8;
    app1 = (app <<(64U-delay));
  }
  Delay_struct->last_part_64 = app1;  
}

//pdmDelay up to 128 PDM samples
static void Delay_3(void * buffer_in, libBeamforming_Handler_Internal * BeamFormingInternal ,TDelay *Delay_struct) 
{
  uint16_t i;	
  uint8_t * buffer_out = BeamFormingInternal->PDM_Delay_Buffer;
  uint16_t jump_in =Delay_struct->jump_in;
  uint16_t nBytesTotal = (uint16_t)BeamFormingInternal->sampling_frequency/8U;
  uint8_t nBytes = Delay_struct->nBytes;
  uint8_t nBits  = Delay_struct->nBits; 
  uint8_t temp;
  uint8_t temp1;
  
  /*Old Last Part in outBuff*/
  for (i = 0; i < nBytes; i++)
  {
    ((uint8_t *)buffer_out)[i] = Delay_struct->last_part[i];    
  }  
  ((uint8_t *)buffer_out)[nBytes] = Delay_struct->last_part[nBytes] & (0xFFU << (8U-nBits));
  
  /*New Last Part*/
  Delay_struct->last_part[nBytes] = ((uint8_t *)buffer_in)[(nBytesTotal - 1U)*jump_in] << (8U - nBits); 
  
  temp1 = ((uint8_t *)buffer_in)[(nBytesTotal - 1U)*jump_in] >> nBits;  
  
  for (i = 1; i < ((uint16_t)nBytes + 1U); i++)
  {
    temp = ((uint8_t *)buffer_in)[(nBytesTotal - 1U - i)*(jump_in)] << (8U - nBits);    
    Delay_struct->last_part[nBytes - i] = temp | temp1 ;    
    temp1 = ((uint8_t *)buffer_in)[(nBytesTotal - 1U - i)*jump_in] >> nBits;
  }   
  
  /*New OutBuff Part*/  
  temp1 = ((uint8_t *)buffer_out)[nBytes] & (0xFF << (8 -nBits));  
  
  for (i = 0; i < (nBytesTotal - nBytes) ; i++)
  {
    temp = ((uint8_t *)buffer_in)[(i)*jump_in] >> nBits;
    ((uint8_t *)buffer_out)[i + nBytes] =  temp | temp1 ;      
    temp1 = ((uint8_t *)buffer_in)[(i)*jump_in] << (8U - nBits);
  }  
}

static void BF_Init(TPCMBF *ParamBf, uint16_t Fs,uint8_t MicChannels, float32_t alpha_antifilter, float32_t gain_antifilter_s1 ,float32_t gain_antifilter_s2 )
{
  ParamBf->OldOut = 0;
  ParamBf->OldIn = 0;
  ParamBf->OldZ = 0;
  ParamBf->HP = 1;
  ParamBf->HP_ALFA = (Fs*256U)/((uint16_t)(2.0*3.14*20.0)+Fs);
  ParamBf->s1_out_old = 0;
  ParamBf->s2_out_old = 0;
  ParamBf->OldOut_s1 = 0;
  ParamBf->OldIn_s1 = 0;
  ParamBf->OldOut_s2 = 0;
  ParamBf->OldIn_s2 = 0;
  ParamBf->alpha_antifilter=(uint16_t)alpha_antifilter;
  ParamBf->gain_antifilter_s1=(uint16_t)gain_antifilter_s1;
  ParamBf->gain_antifilter_s2=(uint16_t)gain_antifilter_s2;
  ParamBf->MicChannels=MicChannels;
}

#ifdef STANDARD_IIR
static void BeamFormer(pcm_sample *ptrBufferIn1, pcm_sample *ptrBufferIn2, pcm_sample *ptrBufferOut, uint32_t nNumSamples, TPCMBF *Param, libBeamforming_Handler_Internal * BeamFormingInternal)
{
  UNUSED(BeamFormingInternal);
  UNUSED(nNumSamples);
  int32_t s1_out;
  int32_t s2_out;
  int16_t Z1;
  int i;
  uint16_t alpha_antifilter=Param->alpha_antifilter;
  uint16_t gain_antifilter_s1=Param->gain_antifilter_s1;
  uint16_t gain_antifilter_s2=Param->gain_antifilter_s2;
  s1_out=Param->s1_out_old;
  s2_out=Param->s2_out_old;
  for (i=0; i<16; i++)
  {
    Z1 = ptrBufferIn1[i];
    s1_out = (alpha_antifilter*s1_out+gain_antifilter_s1*Z1)/256;    
    Z1 = ptrBufferIn2[i]; 
    s2_out = (alpha_antifilter*s2_out+gain_antifilter_s2*Z1)/256;    
    ptrBufferOut[i] = __SSAT(__QSUB(s1_out,s2_out),16);    
  }
  Param->s1_out_old=s1_out;
  Param->s2_out_old=s2_out;
}
#endif

/**
******************************************************************************
* Copyright (C) 2005-2006 Jean-Marc Valin
* File: fftwrap.c
*
* Wrapper for various FFTs
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
*/

static void fft_init(drft_lookup *table, int32_t size)
{
  drft_init((drft_lookup *)table, size);
}

static void fft(drft_lookup *table, float32_t *in, float32_t *out)
{
  int32_t i;
  float32_t scale = 1.0f/((float32_t)(table->n));
  for (i=0;i<((drft_lookup *)table)->n;i++)
  {
    out[i] = scale*in[i];
  }
  drft_forward((drft_lookup *)table, out);
}

static void ifft(drft_lookup *table, float32_t *in, float32_t *out)
{
  int32_t i;
  for (i=0;i<((drft_lookup *)table)->n;i++)
  {
    out[i] = in[i];
  }
  drft_backward((drft_lookup *)table, out);
}


#endif  /*__LIB_BEAMFORMING_C*/

