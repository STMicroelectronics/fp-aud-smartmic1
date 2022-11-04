/**
******************************************************************************
* @file    libSoundSourceLoc.c
* @author  SRA
* @brief   Sound Source Localization core functions
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
#ifndef __LIB_SOUNDSOURCELOC_H
#define __LIB_SOUNDSOURCELOC_H

/* Includes ------------------------------------------------------------------*/
#include "acoustic_sl.h"
#include "arm_math.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
struct _libSoundSourceLoc_Handler_Internal;

typedef struct {
  float32_t  (*SourceLocFunction)(struct _libSoundSourceLoc_Handler_Internal *SLocInternal, int32_t * out_angles);
  uint32_t  (*CheckEventFunction)(struct _libSoundSourceLoc_Handler_Internal *SLocInternal);
}libSoundSourceLoc_Handler_Callbacks;

typedef struct _libSoundSourceLoc_Handler_Internal{
  uint32_t sampling_frequency;
  uint32_t Type;
  uint32_t Buffer_State;
  uint32_t Sample_Number_To_Process;
  uint16_t Sample_Number_Each_ms;
  uint16_t Input_Counter;
  uint16_t EVENT_THRESHOLD;
  uint16_t Mic_Number;
  uint8_t ptr_M1_channels;
  uint8_t ptr_M2_channels;
  uint8_t ptr_M3_channels;
  uint8_t ptr_M4_channels;
  float32_t M12_distance;
  float32_t M34_distance;
  int32_t M12_TAUD;
  int32_t M34_TAUD;
  libSoundSourceLoc_Handler_Callbacks Callbacks;
  int32_t Estimated_Angle_12;
  int32_t Estimated_Angle_34;
  uint32_t resolution;
  void * M1_Data;
  void * M2_Data;
  void * M3_Data;
  void * M4_Data;
  arm_rfft_fast_instance_f32 * SFast;
  float32_t * FFT_Out;
  float32_t * PowerSpectrum;
  float32_t * Phase;
  float32_t * sumArray;
  int32_t * SourceLocFilterArray;
  int32_t * SourceLocFilterArray1;
  float32_t * window;
  
  //dz
  uint32_t        Sample_Number_To_Store;
  uint8_t *       remap_mic; // 4
  uint16_t         num_of_freq;
  uint16_t         num_of_angles;
  uint16_t        freq_range_min;
  uint16_t        freq_range_max;
  float32_t *     mics_distance; // AUDIO_CHANNELS
  float32_t *     mics_angle; // AUDIO_CHANNELS
  float32_t *     theta; // NUM_ANGLES
  float32_t      phi;//
  uint16_t        mics_write_offset; //starting point in every mic \in {0,16,32,...} \subseteq [0 MIC_LEN)
  uint16_t        mics_read_offset; //starting point in every mic \in 0,MIC_LEN
  int16_t *       score_theta;// RESOLUTION
  uint8_t         reactivity;
  uint8_t         stability;
  uint8_t         array_type;// CIRCULAR_ARRAY
  uint8_t         local_stabilizer;
  float32_t *     mic_tmp;//[DFT_LEN]
  float32_t *     ptr_freq_energies;// ptr
  float32_t *     s;//[2*AUDIO_CHANNELS*NUMOF_FREQ]
  int16_t *       frequencies_under_analysis; // NUM OF FREQ
  int16_t *       sources; // 2* OUTPUT SOURCES
  float32_t *     steering_tau;
  
} libSoundSourceLoc_Handler_Internal;

/* Private defines -----------------------------------------------------------*/
#define FFT_POINTS 512

#define LIB_VERSION ((uint32_t)0x00030000)

#define FILTER_MIN 10
#define FILTER_STEP_MAIN 10
#define FILTER_MAX_MAIN 50
#define FILTER_STEP_SEC 8
#define FILTER_MAX_SEC 20

#define FACTOR_INDEX_2_HZ               (uint16_t)(SLocInternal->sampling_frequency/SLocInternal->Sample_Number_To_Process)

#define MIN_RESOLUTION                  4U
#define MAX_NUM_OF_ANGLES               (360U/MIN_RESOLUTION)
#define MAX_NUM_OF_FREQUENCIES          16U
#define MAX_AUDIO_CHANNELS              8U
#define MAX_THRESHOLD                   1000U
#define NUM_OUTPUT_SOURCES              4U
#define CIRCULAR_ARRAY                  1U
#define LINEAR_ARRAY                    2U

#ifndef SOUND_SPEED
#define SOUND_SPEED 	(float32_t)343.1f
#endif

#ifndef UNUSED
#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#endif

#include "doa_via_block_sparsity.c"

/* Private macros ------------------------------------------------------------*/
#define Abs(x) ( (((float32_t)x) < 0.0f) ? (-((float32_t)x)) : ((float32_t)x))
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
#define SaturaL(N, L) (((N)<(L))?(L):(N))
#define SaturaH(N, H) (((N)>(H))?(H):(N))

/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static uint32_t CheckEvent(libSoundSourceLoc_Handler_Internal * SLocInternal);
static float32_t XCORR_GetAngle(libSoundSourceLoc_Handler_Internal * SLocInternal,  int32_t * out_angles);
static float32_t GCC_GetAngle(libSoundSourceLoc_Handler_Internal * SLocInternal, int32_t * out_angles);
static int32_t get_max_pos(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t length,int32_t step);
static void FilterAngle(int32_t *SourceAngle, int32_t* LedStatus, uint16_t max_value, uint16_t A, uint16_t SatA, uint16_t B, uint16_t SatB);

/* Functions Definition ------------------------------------------------------*/
static uint32_t libSoundSourceLoc_GetLibVersion(char *version)
{
  char str1[35] = "ST AcousticSL v3.0.0";
  
  (void)strcpy(version, str1);
  return strlen(str1);
}

static uint32_t libSoundSourceLoc_Init(AcousticSL_Handler_t * pHandler)
{
  libSoundSourceLoc_Handler_Internal * SLocInternal = (libSoundSourceLoc_Handler_Internal *)(pHandler->pInternalMemory);
 (void) memset(pHandler->pInternalMemory, 0 , pHandler->internal_memory_size);
  uint32_t ret = 0;
  
  /* INPUT PTR INTERLEAVING*/
  if(pHandler->ptr_M1_channels > 0U)
  {
    SLocInternal->ptr_M1_channels=pHandler->ptr_M1_channels;
  }
  else
  {
    SLocInternal->ptr_M1_channels=1; /*Set default Value*/
    ret |= ACOUSTIC_SL_PTR_CHANNELS_ERROR;
  }
  
  if(pHandler->ptr_M2_channels > 0U)
  {
    SLocInternal->ptr_M2_channels=pHandler->ptr_M2_channels;
  }
  else
  {
    SLocInternal->ptr_M2_channels=1; /*Set default Value*/
    ret |= ACOUSTIC_SL_PTR_CHANNELS_ERROR;
  }
  
  if(pHandler->ptr_M3_channels > 0U)
  {
    SLocInternal->ptr_M3_channels=pHandler->ptr_M3_channels;
  }
  else
  {
    SLocInternal->ptr_M3_channels=1; /*Set default Value*/
    ret |= ACOUSTIC_SL_PTR_CHANNELS_ERROR;
  }
  
  if(pHandler->ptr_M4_channels > 0U)
  {
    SLocInternal->ptr_M4_channels=pHandler->ptr_M4_channels;
  }
  else
  {
    SLocInternal->ptr_M4_channels=1; /*Set default Value*/
    ret |= ACOUSTIC_SL_PTR_CHANNELS_ERROR;
  }
  
  /* CHANNEL NUMBER*/
  if( (pHandler->channel_number == 2U) || (pHandler->channel_number == 4U) )
  {
    SLocInternal->Mic_Number=(uint16_t)pHandler->channel_number;
  }
  else
  {
    SLocInternal->Mic_Number = 2; /*Set default Value*/
    ret |= ACOUSTIC_SL_CHANNEL_NUMBER_ERROR;
  }
  
  /*ALGORITHM SELECTION*/
  if(   (pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_XCORR)
     || (pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_GCCP)
       || (pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_BMPH))
  {
    SLocInternal->Type=pHandler->algorithm;
  }
  else
  {
    SLocInternal->Type = ACOUSTIC_SL_ALGORITHM_XCORR; /*Set default Value*/
    ret |= ACOUSTIC_SL_ALGORITHM_ERROR;
  }
  
  /*SAMPLING FREQUENCY*/
  if( (pHandler->sampling_frequency == 16000U) || (pHandler->sampling_frequency == 32000U) || (pHandler->sampling_frequency == 48000U))
  {
    SLocInternal->sampling_frequency = pHandler->sampling_frequency;
  }
  else
  {
    SLocInternal->sampling_frequency = 16000; /*Set default Value*/
    ret |= ACOUSTIC_SL_SAMPLING_FREQ_ERROR;
  }
  
  if(SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_XCORR)
  {
    if( pHandler->M12_distance < ((uint16_t)SOUND_SPEED*10000U/SLocInternal->sampling_frequency))
    {
      ret |= ACOUSTIC_SL_DISTANCE_ERROR;
    }
  }
  
  /* NUMBER OF SAMPLES*/
  if (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_XCORR )
  {
    SLocInternal->Sample_Number_To_Process = SLocInternal->sampling_frequency/100U;
    SLocInternal->Sample_Number_To_Store = SLocInternal->sampling_frequency/100U;
  }
  else if (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_BMPH )
  {
    SLocInternal->Sample_Number_To_Store=(uint32_t)pHandler->samples_to_process;
    SLocInternal->Sample_Number_To_Process=(uint32_t)pHandler->samples_to_process/2U;
  }
  else if (SLocInternal->Type== ACOUSTIC_SL_ALGORITHM_GCCP)
  {
    SLocInternal->Sample_Number_To_Store=(uint32_t)pHandler->samples_to_process;
    SLocInternal->Sample_Number_To_Process=(uint32_t)pHandler->samples_to_process;
  }
  else
  {
    /* no other use cases are handled */
  }
  
  if ((SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_GCCP) || (SLocInternal->Type== ACOUSTIC_SL_ALGORITHM_BMPH))
  {
    if( (SLocInternal->Sample_Number_To_Process == 32U)   ||
       (SLocInternal->Sample_Number_To_Process == 64U)   ||
         (SLocInternal->Sample_Number_To_Process == 128U)  ||
           (SLocInternal->Sample_Number_To_Process == 256U)  ||
             (SLocInternal->Sample_Number_To_Process == 512U)  ||
               (SLocInternal->Sample_Number_To_Process == 1024U) ||
                 (SLocInternal->Sample_Number_To_Process == 2048U) ||
                   (SLocInternal->Sample_Number_To_Process == 4096U))
    {
      
    }
    else
    {
      SLocInternal->Sample_Number_To_Process = 256; /*Set default Value*/
      SLocInternal->Sample_Number_To_Store   = 256; /*Set default Value*/
      ret |= ACOUSTIC_SL_NUM_OF_SAMPLES_ERROR;
    }
  }
  
  SLocInternal->Buffer_State = 0;
  SLocInternal->Input_Counter = 0;
  
  SLocInternal->Sample_Number_Each_ms = (uint16_t)SLocInternal->sampling_frequency / 1000U;
  SLocInternal->Callbacks.CheckEventFunction = CheckEvent;
  
  /*SUPPORT VARIABLE USED FOR MEMORY ALLOCATION*/
  volatile uint32_t  byte_offset = sizeof(libSoundSourceLoc_Handler_Internal);
  
  if (SLocInternal->Mic_Number >= 2U)
  {
    /*DISTANCE*/
    if( pHandler->M12_distance > 0U )
    {
      SLocInternal->M12_distance = ((float32_t)(pHandler->M12_distance));
      SLocInternal->M12_distance/=10000.0f;
    }
    else
    {
      SLocInternal->M12_distance = 0.0015f; /*Set default Value*/
      ret |= ACOUSTIC_SL_DISTANCE_ERROR;
    }
    
    if(SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_XCORR)
    {
      SLocInternal->Callbacks.SourceLocFunction = XCORR_GetAngle;
      
      SLocInternal->M1_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Process)*2U*sizeof(int16_t);  //dBuff for M1 size in byte
      
      SLocInternal->M2_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Process)*2U*sizeof(int16_t);  //dBuff for M2 size in byte
      
    }
    else if (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_GCCP)
    {
      SLocInternal->Callbacks.SourceLocFunction = GCC_GetAngle;
      
      SLocInternal->M1_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Process)*2U*sizeof(float32_t);  //dBuff for M1 size in byte
      
      SLocInternal->M2_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Process)*2U*sizeof(float32_t);  //dBuff for M2 size in byte
      
    }
    else
    {
      //follows below
    }
    SLocInternal->M12_TAUD = (int32_t)SLocInternal->M12_distance * ((int32_t)SLocInternal->sampling_frequency/(int32_t)SOUND_SPEED);
  }
  
  if (SLocInternal->Mic_Number == 4U)
  {
    /*DISTANCE*/
    if( pHandler->M34_distance > 0U )
    {
      SLocInternal->M34_distance = ((float32_t)(pHandler->M34_distance));
      SLocInternal->M34_distance/=10000.0f;
    }
    else
    {
      SLocInternal->M34_distance = 0.0015f; /*Set default Value*/
      ret |= ACOUSTIC_SL_DISTANCE_ERROR;
    }
    
    if(SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_XCORR)
    {
      SLocInternal->M3_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Process)*2U*sizeof(int16_t);  //dBuff for M3 size in byte
      
      SLocInternal->M4_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Process)*2U*sizeof(int16_t);  //dBuff for M4 size in byte
    }
    else if (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_GCCP)
    {
      SLocInternal->M3_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Process)*2U*sizeof(float32_t);  //dBuff for M3 size in byte
      
      SLocInternal->M4_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Process)*2U*sizeof(float32_t);  //dBuff for M4 size in byte
    }
    else
    {
      //follows below
    }
    
    SLocInternal->M34_TAUD =(int16_t)floor((float64_t)SLocInternal->M34_distance * ((float64_t)SLocInternal->sampling_frequency/(float64_t)SOUND_SPEED));
  }
  
  if (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_GCCP)
  {
    SLocInternal->SFast=(arm_rfft_fast_instance_f32 *)(((uint8_t *)pHandler->pInternalMemory+byte_offset));
    byte_offset+=sizeof(arm_rfft_fast_instance_f32); /* arm_rfft_instance_f32 size in bytes */
    
    SLocInternal->FFT_Out=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(SLocInternal->Sample_Number_To_Process)*sizeof(float32_t);  /* size of Buff for FFT output in bytes */
    
    SLocInternal->PowerSpectrum=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(SLocInternal->Sample_Number_To_Process)*sizeof(float32_t);  /* size of Buff for PS in bytes */
    
    SLocInternal->Phase=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=182U*sizeof(float32_t);  /* size of Buff for Phase in bytes */
    
    SLocInternal->sumArray=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=182U*sizeof(float32_t);  /* size of Buff for sumArray in bytes */
    
    SLocInternal->window=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=SLocInternal->Sample_Number_To_Process*sizeof(float32_t);  /* size of Buff for sumArray in bytes */
    
    /*Init FFt function*/
    (void)arm_rfft_fast_init_f32(SLocInternal->SFast, (uint16_t)SLocInternal->Sample_Number_To_Process);
    
    /*Init Hamming window*/
    uint32_t i;
    for ( i = 0; i < SLocInternal->Sample_Number_To_Process; i++)
    {
      SLocInternal->window[i]=0.5f*(1.0f-arm_cos_f32((2.0f*PI*(float32_t)i)/((float32_t)SLocInternal->Sample_Number_To_Process-1.0f))); //Hann
    }
  }
  else if (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_BMPH)
  {
    SLocInternal->Callbacks.SourceLocFunction = BCORRF_GetAngle;
    
    if (SLocInternal->Mic_Number >= 1U)
    {
      SLocInternal->M1_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Store)*2U*sizeof(int16_t);  //dBuff for M4 size in byte
    }
    if (SLocInternal->Mic_Number >= 2U)
    {
      SLocInternal->M2_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Store)*2U*sizeof(int16_t);  //dBuff for M4 size in byte
    }
    if (SLocInternal->Mic_Number >= 3U)
    {
      SLocInternal->M3_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Store)*2U*sizeof(int16_t);  //dBuff for M4 size in byte
    }
    if (SLocInternal->Mic_Number >= 4U)
    {
      SLocInternal->M4_Data=(((uint8_t *)pHandler->pInternalMemory+byte_offset));
      byte_offset+=(SLocInternal->Sample_Number_To_Store)*2U*sizeof(int16_t);  //dBuff for M4 size in byte
    }
    SLocInternal->SFast=(arm_rfft_fast_instance_f32 *)(((uint8_t *)pHandler->pInternalMemory+byte_offset));
    byte_offset+=sizeof(arm_rfft_fast_instance_f32); /* arm_rfft_instance_f32 size in bytes */
    
    SLocInternal->FFT_Out=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(SLocInternal->Sample_Number_To_Process)*sizeof(float32_t);  /* size of Buff for FFT output in bytes */
    
    SLocInternal->remap_mic=(uint8_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(MAX_AUDIO_CHANNELS)*sizeof(uint8_t);
    
    SLocInternal->mics_distance=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(MAX_AUDIO_CHANNELS)*sizeof(float32_t);
    
    SLocInternal->mics_angle=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(MAX_AUDIO_CHANNELS)*sizeof(float32_t);
    
    SLocInternal->theta=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(MAX_NUM_OF_ANGLES)*sizeof(float32_t);
    
    SLocInternal->mics_write_offset=0;
    SLocInternal->mics_read_offset=(uint16_t)SLocInternal->Sample_Number_To_Store;
    SLocInternal->score_theta=(int16_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(MAX_NUM_OF_ANGLES)*sizeof(int16_t);
    
    SLocInternal->reactivity=5;
    SLocInternal->stability=25;
    SLocInternal->mic_tmp=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(SLocInternal->Sample_Number_To_Process)*sizeof(float32_t);
    
    SLocInternal->s=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(2U*MAX_AUDIO_CHANNELS*MAX_NUM_OF_FREQUENCIES)*sizeof(float32_t);
    
    SLocInternal->frequencies_under_analysis=(int16_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(MAX_NUM_OF_FREQUENCIES)*sizeof(int16_t);
    
    SLocInternal->sources=(int16_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(2U*NUM_OUTPUT_SOURCES)*sizeof(int16_t);
    
    SLocInternal->array_type=CIRCULAR_ARRAY;
    SLocInternal->local_stabilizer=0;
    SLocInternal->steering_tau=(float32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=(MAX_AUDIO_CHANNELS*MAX_NUM_OF_ANGLES)*sizeof(float32_t);
    
    (void)arm_rfft_fast_init_f32(SLocInternal->SFast, (uint16_t)SLocInternal->Sample_Number_To_Process);
  }
  else
  {
    /* no other use cases are handled */
  }
  
  if ((SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_XCORR) || (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_GCCP))
  {
    SLocInternal->SourceLocFilterArray=(int32_t *)(int32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
    byte_offset+=362U*sizeof(int32_t);  /* size of Buff for SourceLocFilterArray in bytes */
    
    if(SLocInternal->Mic_Number == 4U)
    {
      SLocInternal->SourceLocFilterArray1=(int32_t *)(int32_t *)((uint8_t *)pHandler->pInternalMemory+byte_offset);
      byte_offset+=362U*sizeof(int32_t);  /* size of Buff for SourceLocFilterArray in bytes */
    }
  }
  return ret;
}

static uint32_t libSoundSourceLoc_getMemorySize(AcousticSL_Handler_t * pHandler)
{  
  /*SUPPORT VARIABLE USED FOR MEMORY ALLOCATION*/
  volatile uint32_t  byte_offset = sizeof(libSoundSourceLoc_Handler_Internal);
  
  if (pHandler->channel_number >= 2U)
  {
    if((pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_XCORR))
    {
      byte_offset+=(pHandler->sampling_frequency/100U)*2U*sizeof(int16_t);  //dBuff for M1 size in byte
      byte_offset+=(pHandler->sampling_frequency/100U)*2U*sizeof(int16_t);  //dBuff for M2 size in byte
    }
    else if ((pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_GCCP))
    {
      byte_offset+=((uint32_t)pHandler->samples_to_process)*2U*sizeof(float32_t);  //dBuff for M1 size in byte
      byte_offset+=((uint32_t)pHandler->samples_to_process)*2U*sizeof(float32_t);  //dBuff for M2 size in byte
    }
    else
    {
      //follows below
    }
  }
  
  if (pHandler->channel_number == 4U)
  {
    if((pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_XCORR))
    {
      byte_offset+=(pHandler->sampling_frequency/100U)*2U*sizeof(int16_t);  //dBuff for M3 size in byte
      byte_offset+=(pHandler->sampling_frequency/100U)*2U*sizeof(int16_t);  //dBuff for M4 size in byte
    }
    else if ((pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_GCCP))
    {
      byte_offset+=((uint32_t)pHandler->samples_to_process)*2U*sizeof(float32_t);  //dBuff for M3 size in byte
      byte_offset+=((uint32_t)pHandler->samples_to_process)*2U*sizeof(float32_t);  //dBuff for M4 size in byte
    }
    else
    {
      //follows below
    }
  }
  
  if((pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_GCCP))
  {
    byte_offset+=sizeof(arm_rfft_fast_instance_f32); /* arm_rfft_instance_f32 size in bytes */
    byte_offset+=((uint32_t)pHandler->samples_to_process)*sizeof(float32_t);  /* size of Buff for FFT output in bytes */
    byte_offset+=((uint32_t)pHandler->samples_to_process)*sizeof(float32_t);  /* size of Buff for PS in bytes */
    byte_offset+=182U*sizeof(float32_t);  /* size of Buff for Phase in bytes */
    byte_offset+=182U*sizeof(float32_t);  /* size of Buff for sumArray in bytes */
    byte_offset+=(uint32_t)pHandler->samples_to_process*sizeof(float32_t);  /* size of Buff for Window in bytes */
  }
  
  if (pHandler->channel_number >= 2U)
  {
    if ((pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_BMPH))
    {
      if (pHandler->channel_number == 3U)
      {
        pHandler->channel_number=2;
      }
      byte_offset+=(uint32_t)pHandler->samples_to_process*pHandler->channel_number*2U*sizeof(int16_t);
      byte_offset+=sizeof(arm_rfft_fast_instance_f32); /* arm_rfft_instance_f32 size in bytes */
      byte_offset+=((uint32_t)pHandler->samples_to_process/2U)*sizeof(float32_t);  /* size of Buff for FFT output in bytes */
      byte_offset+=(MAX_AUDIO_CHANNELS)*sizeof(uint8_t);
      byte_offset+=(MAX_AUDIO_CHANNELS)*sizeof(float32_t);
      byte_offset+=(MAX_AUDIO_CHANNELS)*sizeof(float32_t);
      byte_offset+=(MAX_NUM_OF_ANGLES)*sizeof(float32_t);
      byte_offset+=(MAX_NUM_OF_ANGLES)*sizeof(int16_t);
      byte_offset+=((uint32_t)pHandler->samples_to_process/2U)*sizeof(float32_t);
      byte_offset+=(2U*MAX_AUDIO_CHANNELS*MAX_NUM_OF_FREQUENCIES)*sizeof(float32_t);
      byte_offset+=(MAX_NUM_OF_FREQUENCIES)*sizeof(int16_t);
      byte_offset+=(2U*NUM_OUTPUT_SOURCES)*sizeof(int16_t);
      byte_offset+=(MAX_AUDIO_CHANNELS*MAX_NUM_OF_ANGLES)*sizeof(float32_t);
    }
  }
  
  if ((pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_XCORR) || (pHandler->algorithm == ACOUSTIC_SL_ALGORITHM_GCCP))
  {
    byte_offset+=362U*sizeof(int32_t);  /* size of Buff for SourceLocFilterArray in bytes */
    if(pHandler->channel_number == 4U)
    {
      byte_offset+=362U*sizeof(int32_t);  /* size of Buff for SourceLocFilterArray in bytes */
    }
  }
  
  while((++byte_offset%4U)!=0U)
  {
  }
  
  pHandler->internal_memory_size= byte_offset;
  return 0;
}

static uint32_t libSoundSourceLoc_Data_Input(void *pM1, void *pM2, void *pM3, void *pM4, AcousticSL_Handler_t * pHandler)
{  
  libSoundSourceLoc_Handler_Internal * SLocInternal = (libSoundSourceLoc_Handler_Internal *)(pHandler->pInternalMemory);
  
  uint8_t ret = 0;
  uint32_t i;
  if((SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_XCORR) || (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_BMPH) )
  {
    for (i = 0; i < SLocInternal->Sample_Number_Each_ms; i ++)
    {
      ((int16_t *)(SLocInternal->M1_Data))[SLocInternal->Input_Counter] = ((int16_t *)(pM1))[i*SLocInternal->ptr_M1_channels];
      ((int16_t *)(SLocInternal->M2_Data))[SLocInternal->Input_Counter] = ((int16_t *)(pM2))[i*SLocInternal->ptr_M2_channels];
      if (SLocInternal->Mic_Number == 4U)
      {
        ((int16_t *)(SLocInternal->M3_Data))[SLocInternal->Input_Counter] = ((int16_t *)(pM3))[i*SLocInternal->ptr_M3_channels];
        ((int16_t *)(SLocInternal->M4_Data))[SLocInternal->Input_Counter] = ((int16_t *)(pM4))[i*SLocInternal->ptr_M4_channels];
      }
      SLocInternal->Input_Counter ++;
    }
  }
  else if(SLocInternal->Type ==  ACOUSTIC_SL_ALGORITHM_GCCP )
  {
    for (i = 0; i < SLocInternal->Sample_Number_Each_ms; i ++)
    {
      ((float32_t *)(SLocInternal->M1_Data))[SLocInternal->Input_Counter] = (float32_t)((int16_t *)(pM1))[i*SLocInternal->ptr_M1_channels];//todo interleaving
      ((float32_t *)(SLocInternal->M2_Data))[SLocInternal->Input_Counter] = (float32_t)((int16_t *)(pM2))[i*SLocInternal->ptr_M2_channels];
      if (SLocInternal->Mic_Number == 4U)
      {
        ((float32_t *)(SLocInternal->M3_Data))[SLocInternal->Input_Counter] = (float32_t)((int16_t *)(pM3))[i*SLocInternal->ptr_M3_channels];
        ((float32_t *)(SLocInternal->M4_Data))[SLocInternal->Input_Counter] = (float32_t)((int16_t *)(pM4))[i*SLocInternal->ptr_M4_channels];
      }
      SLocInternal->Input_Counter ++;
    }
  }
  else
  {
    /* no other use cases are handled */
  }
  
  if(SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_BMPH)
  {
    if(SLocInternal->Input_Counter == SLocInternal->Sample_Number_To_Store)
    {
      ret = 1;
      SLocInternal->Buffer_State = 1;
      SLocInternal->mics_read_offset= 0;
    }
    if(SLocInternal->Input_Counter == (SLocInternal->Sample_Number_To_Store * 2U))
    {
      ret = 1;
      SLocInternal->Buffer_State = 2;
      SLocInternal->Input_Counter = 0;
      SLocInternal->mics_read_offset = (uint16_t)SLocInternal->Sample_Number_To_Store;
    }
  }
  else
  {
    if(SLocInternal->Input_Counter == SLocInternal->Sample_Number_To_Store)
    {
      ret = 1;
      SLocInternal->Buffer_State = 1;
    }
    if(SLocInternal->Input_Counter == (SLocInternal->Sample_Number_To_Store * 2U))
    {
      ret = 1;
      SLocInternal->Buffer_State = 2;
      SLocInternal->Input_Counter = 0;
    }
  }
  
  return ret;
}

static uint32_t libSoundSourceLoc_Process(int32_t * Estimated_Angle, AcousticSL_Handler_t * pHandler)
{
  libSoundSourceLoc_Handler_Internal * SLocInternal = (libSoundSourceLoc_Handler_Internal *)(pHandler->pInternalMemory);
  
  int32_t Estimated_temp_360[2];
  
  if(SLocInternal->Buffer_State!=0U)
  {
    if((SLocInternal->Type==ACOUSTIC_SL_ALGORITHM_BMPH) || (SLocInternal->Callbacks.CheckEventFunction(SLocInternal)==1))
    {
      SLocInternal->Callbacks.SourceLocFunction(SLocInternal,Estimated_temp_360);      
    }
    else
    {
      Estimated_temp_360[0]=-1;
    }
  }
  
  if (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_BMPH)
  {
    if (Estimated_temp_360[0]==-500)
    {
      Estimated_temp_360[0]=-100;
    }
  }
  else if(SLocInternal->Mic_Number==4U)
  {
    /*TODO: filter on the index instead (before angle computation)*/
    FilterAngle((int32_t *)&Estimated_temp_360[0], SLocInternal->SourceLocFilterArray, 360 , FILTER_STEP_MAIN , FILTER_MAX_MAIN, FILTER_STEP_SEC ,FILTER_MAX_SEC);
    
    if(Estimated_temp_360[0]==-1)
    {
      Estimated_temp_360[0]=-100;
    }
  }
  else
  {
    FilterAngle((int32_t *)&Estimated_temp_360[0], SLocInternal->SourceLocFilterArray, 180 , 10 , 50, 8 ,20);
    if(Estimated_temp_360[0]==-1)
    {
      Estimated_temp_360[0]=-100;
    }
    else
    {
      Estimated_temp_360[0]-=90;
    }
  }
  
  Estimated_Angle[0] = Estimated_temp_360[0];
  return 0;
}

static uint32_t libSoundSourceLoc_setConfig(AcousticSL_Handler_t * pHandler, AcousticSL_Config_t * pConfig)
{
  libSoundSourceLoc_Handler_Internal * SLocInternal = (libSoundSourceLoc_Handler_Internal *)(pHandler->pInternalMemory);
  
  uint32_t ret = 0;
  
  /*RESOLUTION*/
  if( (pConfig->resolution > 0U) && (pConfig->resolution < 46U) )
  {
    SLocInternal->resolution = pConfig->resolution;
  }
  else if ( SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_GCCP)
  {
    SLocInternal->resolution = MIN_RESOLUTION; /*Set default Value*/
    ret |= ACOUSTIC_SL_RESOLUTION_ERROR;
  }
  else
  {
    /* no other use cases are handled */
  }
  
  if ( (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_BMPH) && (pConfig->resolution < MIN_RESOLUTION) )
  {
    SLocInternal->resolution = MIN_RESOLUTION; /*Set default Value*/
    ret |= ACOUSTIC_SL_RESOLUTION_ERROR;
  }
  
  /*THRESHOLD*/
  if(pConfig->threshold <= MAX_THRESHOLD)
  {
    SLocInternal->EVENT_THRESHOLD = pConfig->threshold;
  }
  else
  {
    SLocInternal->EVENT_THRESHOLD = 24; /*Set default Value*/
    ret |= ACOUSTIC_SL_THRESHOLD_ERROR;
  }
  
  if (SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_BMPH)
  {
    if ( SLocInternal->Mic_Number == 2U )
    {
      SLocInternal->array_type=LINEAR_ARRAY;
    }
    
    MicsArray_init(SLocInternal);
    DicretizedAngles_init(SLocInternal);
    Frequency_init(SLocInternal);
    SteeringMatrix_init(SLocInternal);
  }
  
  return ret;
}

static uint32_t libSoundSourceLoc_getConfig(AcousticSL_Handler_t * pHandler, AcousticSL_Config_t * pConfig)
{
  libSoundSourceLoc_Handler_Internal * SLocInternal = (libSoundSourceLoc_Handler_Internal *)(pHandler->pInternalMemory);
  
  pConfig->resolution=SLocInternal->resolution;
  pConfig->threshold=SLocInternal->EVENT_THRESHOLD;
  return 0;
}

static uint32_t CheckEvent(libSoundSourceLoc_Handler_Internal * SLocInternal)
{  
  uint32_t ret = 0;
  uint32_t a1,b;
  a1=0;
  for(b=0;b<SLocInternal->Sample_Number_To_Process;b++)
  {
    if(SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_XCORR)
    {
      a1=a1+(uint32_t)Abs(((int16_t *)(SLocInternal->M1_Data))[((SLocInternal->Buffer_State-1)*SLocInternal->Sample_Number_To_Process)+b]);
    }
    else if(SLocInternal->Type == ACOUSTIC_SL_ALGORITHM_GCCP)
    {
      a1=a1+(uint32_t)Abs(((float32_t *)(SLocInternal->M1_Data))[((SLocInternal->Buffer_State-1)*SLocInternal->Sample_Number_To_Process)+b]);
    }
    else
    {
      /* no other use cases are handled */
    }
  }
  a1=a1/SLocInternal->Sample_Number_To_Process;
  
  if( a1> ( (uint32_t)SLocInternal->EVENT_THRESHOLD * ((32768U / MAX_THRESHOLD) + 1U) ) )  // [0, 32768] -> [0 ,MAX_THRESHOLD ]
  {
    ret = 1;
  }
  return ret;
}

static float32_t XCORR_GetAngle(libSoundSourceLoc_Handler_Internal * SLocInternal,  int32_t * out_angles)
{
  int32_t tau, k;
  float32_t delta_t24 = 0.0f;
  float32_t delta_t13 = 0.0f;
  float32_t test = 0.0f;
  uint32_t correlation, MAX13, MAX24;
  if(SLocInternal->Mic_Number >= 2U)
  {
    MAX13=0x80000000U;  //-inf
    for(tau=-SLocInternal->M12_TAUD;tau<=SLocInternal->M12_TAUD;tau++)
    {
      correlation=0;
      for(k=SLocInternal->M12_TAUD; k<((int32_t)SLocInternal->Sample_Number_To_Process-SLocInternal->M12_TAUD);k++)
      {
        correlation = correlation + (((uint32_t)((int16_t *)(SLocInternal->M2_Data))[((SLocInternal->Buffer_State-1U)*SLocInternal->Sample_Number_To_Process)+(uint32_t)k] * (uint32_t)((int16_t *)(SLocInternal->M1_Data))[((SLocInternal->Buffer_State-1U)*SLocInternal->Sample_Number_To_Process)+(uint32_t)k+(uint32_t)tau])/256U);
      }
      if (correlation > MAX13)
      {
        MAX13 = correlation;
        delta_t13=(float32_t)tau;
      }
    }
  }
  if(SLocInternal->Mic_Number == 4U)
  {
    MAX24=0x80000000U;  //-inf
    for(tau=-SLocInternal->M34_TAUD;tau<=SLocInternal->M34_TAUD;tau++)
    {
      correlation=0;
      for(k=SLocInternal->M34_TAUD; k<((int32_t)SLocInternal->Sample_Number_To_Process-SLocInternal->M34_TAUD); k++)
      {
        correlation = correlation + (((uint32_t)((int16_t *)(SLocInternal->M4_Data))[((SLocInternal->Buffer_State-1U)*SLocInternal->Sample_Number_To_Process)+(uint32_t)k] * (uint32_t)((int16_t *)(SLocInternal->M3_Data))[((SLocInternal->Buffer_State-1U)*SLocInternal->Sample_Number_To_Process)+(uint32_t)k+(uint32_t)tau])/256U);
      }
      if (correlation > MAX24)
      {
        MAX24 = correlation;
        delta_t24=(float32_t)tau;
      }
    }
  }
  SLocInternal->Buffer_State=0;
  if(SLocInternal->Mic_Number == 2U)
  {
    test = (delta_t13+(float32_t)SLocInternal->M12_TAUD )* (180.0f/((float32_t)SLocInternal->M12_TAUD*2.0f));
  }
  else if(SLocInternal->Mic_Number == 4U)
  {
    test=(float32_t)atan2(delta_t13, delta_t24);
    test= test * (360.0f / (3.14f*2.0f) );
    if(test<0.0f)
    {
      test+=360.0f;    /* in [0 360] -> TODO: Optimize the whole angle computation from the maximum phase index*/
    }
  }
  else
  {
    /* no other use cases are handled */
  }
  
  out_angles[0]=(int32_t)test;
  return test;
}

static float32_t GCC_GetAngle(libSoundSourceLoc_Handler_Internal * SLocInternal, int32_t * out_angles)
{
  uint32_t j;
  float32_t fi = 0.0f;;
  uint32_t buffer_offset = (SLocInternal->Buffer_State-1U)*SLocInternal->Sample_Number_To_Process;
  float32_t * M1_Data = (float32_t *)SLocInternal->M1_Data;
  float32_t * M2_Data = (float32_t *)SLocInternal->M2_Data;
  float32_t * M3_Data = (float32_t *)SLocInternal->M3_Data;
  float32_t * M4_Data = (float32_t *)SLocInternal->M4_Data;
  float32_t * Power_Spectrum = (float32_t *)SLocInternal->PowerSpectrum;
  float32_t * FFT_Out = (float32_t *)SLocInternal->FFT_Out;
  float32_t * hanning = (float32_t *)SLocInternal->window;
  float32_t tempMag = 1e-7f;
  /*WINDOWING*/
  for(j=0;j<SLocInternal->Sample_Number_To_Process;j++)
  {
    if(SLocInternal->Mic_Number >= 2U)
    {
      M1_Data[buffer_offset + j] *= hanning[j];
      M2_Data[buffer_offset + j] *= hanning[j];
    }
    if(SLocInternal->Mic_Number == 4U)
    {
      M3_Data[buffer_offset + j] *= hanning[j];
      M4_Data[buffer_offset + j] *= hanning[j];
    }
  }
  /*FFTs*/
  if(SLocInternal->Mic_Number >= 2U)
  {
    /*M1 FFT*/
    arm_rfft_fast_f32(SLocInternal->SFast,&M1_Data[buffer_offset],(float32_t *)FFT_Out,0);
    for (j=0;j<SLocInternal->Sample_Number_To_Process;j++)
    {
      M1_Data[buffer_offset + j]=((float32_t *)FFT_Out)[j];
    }
    /*M2 FFT*/
    arm_rfft_fast_f32(SLocInternal->SFast,&M2_Data[buffer_offset],(float32_t *)FFT_Out,0);
    for (j=0;j<SLocInternal->Sample_Number_To_Process;j++)
    {
      M2_Data[buffer_offset + j]=((float32_t *)FFT_Out)[j];
    }
  }
  if(SLocInternal->Mic_Number == 4U)
  {
    /*M3 FFT*/
    arm_rfft_fast_f32(SLocInternal->SFast,&M3_Data[buffer_offset],(float32_t *)FFT_Out,0);
    for (j=0;j<SLocInternal->Sample_Number_To_Process;j++)
    {
      M3_Data[buffer_offset + j]=((float32_t *)FFT_Out)[j];
    }
    /*M4 FFT*/
    arm_rfft_fast_f32(SLocInternal->SFast,&M4_Data[buffer_offset],(float32_t *)FFT_Out,0);
    for (j=0;j<SLocInternal->Sample_Number_To_Process;j++)
    {
      M4_Data[buffer_offset + j]=((float32_t *)FFT_Out)[j];
    }
  }
  
  /*FILTERING*/
  if(SLocInternal->Mic_Number >= 2U)
  {
    M1_Data[buffer_offset]=0.0f;
    M2_Data[buffer_offset]=0.0f;
    
    M1_Data[buffer_offset+1U]=0.0f;
    M2_Data[buffer_offset+1U]=0.0f;
  }
  if(SLocInternal->Mic_Number == 4U)
  {
    M3_Data[buffer_offset]=0.0f;
    M4_Data[buffer_offset]=0.0f;
    
    M3_Data[buffer_offset+1U]=0.0f;
    M4_Data[buffer_offset+1U]=0.0f;
  }
  
  if(SLocInternal->Mic_Number >= 2U)
  {
    /*POWER SPECTRUM*/
    for(j=0; j<(SLocInternal->Sample_Number_To_Process/8U); j++)
    {
      Power_Spectrum[(j*2U)] = (M2_Data[buffer_offset+(j*2U)] * M1_Data[buffer_offset+(j*2U)]) + (M2_Data[buffer_offset+(j*2U)+1U] * M1_Data[buffer_offset+(j*2U)+1U]);
      Power_Spectrum[(j*2U)+1U] = (-((M2_Data[buffer_offset+(j*2U)])*(M1_Data[buffer_offset+(j*2U)+1U]))) + (M2_Data[buffer_offset+(j*2U)+1U] * M1_Data[buffer_offset+(j*2U)]);
      
      float32_t arg_sqrtf = (Power_Spectrum[(j*2U)]*Power_Spectrum[(j*2U)]) + (Power_Spectrum[(j*2U)+1U]*Power_Spectrum[(j*2U)+1U]);
      if (arg_sqrtf >= 0.0f)
      {
        tempMag=sqrtf(arg_sqrtf);      
        if(tempMag<(1e-7f))
        {
          tempMag=(1e-7f);
        }
      }
      Power_Spectrum[(j*2U)]=(Power_Spectrum[(j*2U)])/tempMag;
      Power_Spectrum[(j*2U)+1U]=(Power_Spectrum[(j*2U)+1U])/tempMag;
    }
    
    int32_t angle;
    float32_t anglesNum=(180.0f/(float32_t)SLocInternal->resolution);
    
    for(angle=0;angle<(int32_t)anglesNum;angle++)
    {
      float32_t idftSum=0.0f;
      int32_t bin;
      float32_t theta;
      float32_t cosineTerm;
      float32_t sineTerm;
      float32_t theta_term= -2.0f*PI*(float32_t)SLocInternal->sampling_frequency*SLocInternal->M12_distance*(float32_t)arm_cos_f32(PI - ((PI*(float32_t)angle)/(anglesNum-1.0f)))/((float32_t)SLocInternal->Sample_Number_To_Process*SOUND_SPEED);
      for(bin=0; bin<((int32_t)SLocInternal->Sample_Number_To_Process/8); bin++)
      {
        theta = theta_term*(float32_t)bin;
        cosineTerm=arm_cos_f32(theta);
        sineTerm=arm_sin_f32(theta);
        idftSum+=(Power_Spectrum[2*bin]*cosineTerm)-(Power_Spectrum[(2*bin)+1]*sineTerm);
      }
      SLocInternal->Phase[angle]=idftSum;
    }
    
    SLocInternal->Estimated_Angle_12=get_max_pos(SLocInternal,(int32_t)anglesNum,(((int32_t)anglesNum/40)+1));
    SLocInternal->Estimated_Angle_12= 180-(int32_t)floor((180.0/((float64_t)anglesNum*2.0))+((float64_t)SLocInternal->Estimated_Angle_12*(float64_t)(SLocInternal->resolution)));
  }
  if(SLocInternal->Mic_Number == 4U)
  {
    /*POWER SPECTRUM*/
    for(j=0; j<(SLocInternal->Sample_Number_To_Process/8U); j++)
    {
      Power_Spectrum[(j*2U)] = (M4_Data[buffer_offset+(j*2U)] * M3_Data[buffer_offset+(j*2U)]) + (M4_Data[buffer_offset+(j*2U)+1U] * M3_Data[buffer_offset+(j*2U)+1U]);
      Power_Spectrum[(j*2U)+1U] = (-((M4_Data[buffer_offset+(j*2U)])*(M3_Data[buffer_offset+(j*2U)+1U]))) + (M4_Data[buffer_offset+(j*2U)+1U] * M3_Data[buffer_offset+(j*2U)]);
      
      float32_t arg_sqrtf = (Power_Spectrum[(j*2U)]*Power_Spectrum[(j*2U)]) + (Power_Spectrum[(j*2U)+1U]*Power_Spectrum[(j*2U)+1U]);
      if (arg_sqrtf >= 0.0f)
      {
        tempMag=sqrtf(arg_sqrtf);      
        if(tempMag<(1e-7f))
        {
          tempMag=(1e-7f);
        }
      }
      Power_Spectrum[(j*2U)]=(Power_Spectrum[(j*2U)])/tempMag;
      Power_Spectrum[(j*2U)+1U]=(Power_Spectrum[(j*2U)+1U])/tempMag;
    }
    
    int32_t angle;
    float32_t anglesNum=(180.0f/(float32_t)SLocInternal->resolution);
    
    for(angle=0;angle<(int32_t)anglesNum;angle++)
    {
      float32_t idftSum=0.0f;
      int32_t bin;
      float32_t theta;
      float32_t cosineTerm;
      float32_t sineTerm;
      float32_t theta_term= -2.0f*PI*(float32_t)SLocInternal->sampling_frequency*SLocInternal->M34_distance*(float32_t)arm_cos_f32(PI - ((PI*(float32_t)angle)/(anglesNum-1.0f)))/((float32_t)SLocInternal->Sample_Number_To_Process*SOUND_SPEED);
      for(bin=0; bin<((int32_t)SLocInternal->Sample_Number_To_Process/8); bin++)
      {
        theta = theta_term*(float32_t)bin;
        cosineTerm=arm_cos_f32(theta);
        sineTerm=arm_sin_f32(theta);
        idftSum+=(Power_Spectrum[2*bin]*cosineTerm)-(Power_Spectrum[(2*bin)+1]*sineTerm);
      }
      SLocInternal->Phase[angle]=idftSum;
    }
    
    SLocInternal->Estimated_Angle_34=get_max_pos(SLocInternal,(int32_t)anglesNum,(((int32_t)anglesNum/40)+1));
    SLocInternal->Estimated_Angle_34= 180-(int32_t)floor((180.0/((float64_t)anglesNum*2.0))+((float64_t)SLocInternal->Estimated_Angle_34*(float64_t)(SLocInternal->resolution)));
  }
  float32_t angle_out_f = 0.0f;
  
  if(SLocInternal->Mic_Number == 2U)
  {
    angle_out_f=(float32_t)SLocInternal->Estimated_Angle_12;
  }
  else if (SLocInternal->Mic_Number == 4U)
  {
    if((SLocInternal->Estimated_Angle_12 + SLocInternal->Estimated_Angle_34) < (90 - ((int32_t)SLocInternal->resolution * 2)))
    {
      out_angles[0]=-1;
      out_angles[1]=-1;
      return -1.0f;
    }
    else
    {
      float32_t angle1_f = arm_cos_f32((((float32_t)SLocInternal->Estimated_Angle_12)/180.0f)*3.14f);
      float32_t angle2_f = arm_cos_f32((((float32_t)SLocInternal->Estimated_Angle_34)/180.0f)*3.14f);
      float32_t cos_fi = 0.0f;
      float32_t arg_cos_fi = 1.0f - (angle1_f*angle1_f) - (angle2_f*angle2_f);
      
      if (arg_cos_fi >= 0.0f)
      {
        cos_fi = sqrtf(arg_cos_fi);
      }
      angle_out_f = atan2f((angle1_f),(angle2_f));
      angle_out_f= angle_out_f * (360.0f / (3.14f*2.0f));
      if(angle_out_f < 0.0f)
      {
        angle_out_f+=360.0f;    /* in [0 360] -> TODO: Optimize the whole angle computation from the maximum phase index*/
      }
      
      if ((cos_fi <= 1.0f) && (cos_fi >= -1.0f))
      {
        fi = acosf(cos_fi)*180.0f/3.14f;
      }
    }
  }
  
  SLocInternal->Buffer_State=0;
  out_angles[0]=(int32_t)angle_out_f;
  
  if(isnan(fi)!= 0U)
  {
    out_angles[1]=(int32_t)fi;
  }
  else
  {
    out_angles[1]=-1;
  }
  
  return angle_out_f;
}

static int32_t get_max_pos(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t length,int32_t step) 
{
  float32_t * array = SLocInternal->Phase;
  float32_t * sumArray = SLocInternal->sumArray;
  float32_t maxValue=array[0];
  int32_t maxPos=0;
  float32_t maxSum;
  int32_t i;
  
  for ( i=0; i<(length-(step-1)); i++)
  {
    maxSum=0.0f;
    int32_t j;
    for( j=0;j<step;j++)
    {
      maxSum+=array[i+j];
    }
    sumArray[i]=maxSum;
  }
  
  for (i=0; i<(length-(step-1)); i++)
  {
    if (maxValue<(sumArray[i]))
    {
      maxValue=(sumArray[i]);
      maxPos=i+(step/2);
    }
  }
  return maxPos;
}

/*LedStatus is a 32 bit signed array of minimum dimension = max_value + 1*/
/*Source Angle is a number from 0 to max_value and -1 is for no source*/
static void FilterAngle(int32_t *SourceAngle, int32_t* LedStatus, uint16_t max_value, uint16_t A, uint16_t SatA, uint16_t B, uint16_t SatB)
{
  uint16_t S1, S2, i, SoundDirIdx;
  int32_t SoundDir;
  int16_t max = 0;
  int16_t index_max = (int16_t)max_value + 1;
  
  if(*SourceAngle==-1)
  {
    SoundDir=0;
  }
  else
  {
    SoundDir = ((*SourceAngle)+1);
  }
  *SourceAngle = SoundDir;
  if (SoundDir!=0)
  {
    SoundDir--;
    SoundDirIdx = (uint16_t)SoundDir;
    
    S1=(SoundDirIdx==0U) ? max_value : (SoundDirIdx-1U);
    S2=(SoundDirIdx==max_value) ? 0U : (SoundDirIdx+1U);
    LedStatus[SoundDirIdx]=SaturaH(LedStatus[SoundDirIdx] + (int32_t)A, (int32_t)SatA);
    LedStatus[S1]=SaturaH(LedStatus[S1] + (int32_t)B, (int32_t)SatB);
    LedStatus[S2]=SaturaH(LedStatus[S2] + (int32_t)B, (int32_t)SatB);
  }
  
  for (i=0; i<(max_value+1U); i++)
  {
    LedStatus[i]=SaturaL(LedStatus[i]-1, 0);
    if ((LedStatus[i]>7) && (LedStatus[i]>(int32_t)max))
    {
      max = (int16_t)LedStatus[i];
      index_max=(int16_t)i;
    }
  }
  if(index_max == ((int16_t)max_value + 1))
  {
    *SourceAngle = -1;
  }
  else
  {
    *SourceAngle = (index_max);
  }
}


#endif /*__LIB_SOUNDSOURCELOC_H*/

