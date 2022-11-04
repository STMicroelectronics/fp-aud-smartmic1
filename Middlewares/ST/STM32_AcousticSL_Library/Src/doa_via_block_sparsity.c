/**
******************************************************************************
* @file    doa_via_block_sparsity.c
* @author  SRA
* @brief   Sound Source Localization based on block sparsity algorithm
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
#ifndef __DOA_VIA_BLOCK_SPARSITY_H
#define __DOA_VIA_BLOCK_SPARSITY_H

#ifndef SOUND_SPEED
#define SOUND_SPEED 	                (float32_t)343.1
#endif

#ifndef NUM_OUTPUT_SOURCES              
#define NUM_OUTPUT_SOURCES              4U
#endif

#define PI_2                            6.28318531f
#define PI_3_2                          4.71238898f
#define PI_1_2                          1.57079633f
#define PI_1_6                          0.52359878f
#define FACTOR_RAD_2_DEG                57.2957795f

#define MIN(a,b)                        ( ((a)>(b)) ? (b) : (a) )
#define SQR(a)                          ((a)*(a))
#define ABS(a)                          ( (((float32_t)a) < 0.0f) ? (-((float32_t)a)) : ((float32_t)a))
#define MOD(a, m)                       ( (a+m)%m )
#define USE_M1_MEM                      &( ((q15_t*)(SLocInternal->M1_Data)) [SLocInternal->mics_read_offset] )
#define USE_M2_MEM                      &( ((q15_t*)(SLocInternal->M2_Data)) [SLocInternal->mics_read_offset] )



#include "arm_math.h"



static void MicsArray_init(libSoundSourceLoc_Handler_Internal * SLocInternal);
static void DicretizedAngles_init(libSoundSourceLoc_Handler_Internal * SLocInternal);
static void Frequency_init(libSoundSourceLoc_Handler_Internal * SLocInternal);

static float32_t BCORRF_GetAngle(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t* out_angles);

static int32_t vad(libSoundSourceLoc_Handler_Internal * SLocInternal);
static void DynamicFrequenciesSelection(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t starting_index);
static void ExtractFrequenciesOfInterest(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t starting_index);
static int16_t StableDoA(libSoundSourceLoc_Handler_Internal * SLocInternal, int32_t i_max, float32_t v_max);
static void max_block_correlation(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t *i_max, float32_t *v_max);

static void sort_desc_f32(float32_t *a, int32_t len, int16_t *p);
static void sort_desc_i16(int16_t *a, int32_t len, int16_t *p);
static void ComputeDFT(libSoundSourceLoc_Handler_Internal * SLocInternal, void * M_Data,int32_t starting_index);
static int16_t adjust_output_angle(float32_t theta, uint8_t transform);

static float32_t dz_sin_f32(float32_t x);
static float32_t dz_cos_f32(float32_t x);
static void dz_cos_msin_f32_table(float32_t x,float32_t* s);

/* dz: Generate the array of microphones. */
static void MicsArray_init(libSoundSourceLoc_Handler_Internal * SLocInternal)
{  
  uint16_t i;
  
  if (SLocInternal->array_type == LINEAR_ARRAY )
  {  
    //equispaced linear array of mics with midpoint in 0.
    for (i=0;i<(SLocInternal->Mic_Number/2U);i++)
    {
      SLocInternal->mics_distance[i] = ((((float32_t)SLocInternal->Mic_Number-1.0f)*0.5f) - (float32_t)i) * SLocInternal->M12_distance;
      SLocInternal->mics_distance[SLocInternal->Mic_Number-1U-i] = SLocInternal->mics_distance[i] ;
      SLocInternal->mics_angle[i] = 0.0f;
      SLocInternal->mics_angle[SLocInternal->Mic_Number-1U-i] = PI;
    }
    if ((SLocInternal->Mic_Number%2U) == 1U)
    {
      SLocInternal->mics_angle[SLocInternal->Mic_Number/2U] = 0.0f;
      SLocInternal->mics_distance[SLocInternal->Mic_Number/2U] = 0.0f;
    }    
  }
  else if (SLocInternal->array_type == CIRCULAR_ARRAY)
  {    
    //equispaced circular array of mics with center in 0.
    for (i=0;i<SLocInternal->Mic_Number;i++)
    {
      SLocInternal->mics_angle[i]=((float32_t)(i)*2.0f*PI)/(float32_t)SLocInternal->Mic_Number;
      SLocInternal->mics_distance[i]= SLocInternal->M12_distance / 2.0f;
    }
  }
  else
  {
    /* No other use cases are handled*/
  }
  
  for (i=0;i<SLocInternal->Mic_Number;i++)
  {
    SLocInternal->remap_mic[i]=(uint8_t)i;
  }
  
  if(SLocInternal->Mic_Number == 4U)
  {
    SLocInternal->remap_mic[0]=0;
    SLocInternal->remap_mic[1]=2;
    SLocInternal->remap_mic[2]=1;
    SLocInternal->remap_mic[3]=3;
  }
  
  return;
}


/* Generate the angles of arrivals of the virtual sources. */
static void DicretizedAngles_init(libSoundSourceLoc_Handler_Internal * SLocInternal)
{  
  uint16_t n;
  
  if (SLocInternal->array_type == LINEAR_ARRAY /*|| SLocInternal->Mic_Number == 2 */)
  {  
    // [0,180] -> correction [30 150]=[pi/6; pi*5/6]
    SLocInternal->num_of_angles = (uint16_t) ((180U/SLocInternal->resolution) + 1U);
    for(n=0; n<SLocInternal->num_of_angles; n++)
    {
      SLocInternal->theta[n] = (PI/6.0f) + ((float32_t)n*(PI/3.0f*2.0f/((float32_t)SLocInternal->num_of_angles-1.0f)));
    }
    SLocInternal->phi = PI/2.0f;
    
  }
  else if (SLocInternal->array_type == CIRCULAR_ARRAY)
  {
    // [0,360)
    SLocInternal->num_of_angles = (uint16_t)(360U/SLocInternal->resolution);
    for(n=0;n<SLocInternal->num_of_angles;n++)
    {
      SLocInternal->theta[n] = (float32_t)n*2.0f*(PI/(float32_t)SLocInternal->num_of_angles);
    }
    SLocInternal->phi = PI/8.0f*3.0f;    
  }
  else
  {
    /* No other use cases are handled*/
  }
  
  (void)memset(SLocInternal->score_theta,0,2U*(uint32_t)SLocInternal->num_of_angles*sizeof(int16_t));
}


static void Frequency_init(libSoundSourceLoc_Handler_Internal * SLocInternal)
{
  SLocInternal->num_of_freq=5;
  SLocInternal->freq_range_min=200U/FACTOR_INDEX_2_HZ;
  SLocInternal->freq_range_max=3000U/FACTOR_INDEX_2_HZ;  
}

static void SteeringMatrix_init(libSoundSourceLoc_Handler_Internal * SLocInternal)
{  
  float32_t sin_pn_c=dz_sin_f32( SLocInternal->phi )/SOUND_SPEED;
  uint16_t n,m;
  
  for (n=0;n<SLocInternal->num_of_angles;n++)
  {
    for (m=0;m<SLocInternal->Mic_Number;m++)
    {
      float32_t tmp;
      tmp = SLocInternal->mics_distance[m] * dz_cos_f32( SLocInternal->theta[n]-SLocInternal->mics_angle[m] ) * sin_pn_c;
      SLocInternal->steering_tau[(n*SLocInternal->Mic_Number)+m] = tmp;      
    }
  }  
}


/* Estimate the directions of arrival. */
static float32_t BCORRF_GetAngle(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t *out_angles)
{  
  int32_t i_max=-1;
  float32_t v_max=-1.0f; 
  
  /* Voice Activity Detection */
  int32_t starting_index =vad(SLocInternal);
  
  /* Source Localization */
  if(starting_index>=0)
  {    
    //get frequencies of interest
    DynamicFrequenciesSelection(SLocInternal,starting_index);    
    //azimuthal estimation
    max_block_correlation(SLocInternal,&i_max,&v_max);
  }
  
  /* Stable DoA estimation */
  *out_angles=StableDoA(SLocInternal, i_max, v_max);
  
  return 0.0f;  
}


/* dz: Voice-activity detector*/
//analyzes only the microphone mic_index
static int32_t vad(libSoundSourceLoc_Handler_Internal * SLocInternal)
{  
  uint32_t i;
  int32_t ret = -1;
  int32_t starting_index = 0;
  
  q15_t *mic_tmp;
  mic_tmp=(q15_t*)&(((q15_t*)(SLocInternal->M1_Data))[  SLocInternal->mics_read_offset ]);
  
  q63_t max_power, power;
  
  //find contiguous segment of maximal power
  arm_power_q15(mic_tmp,SLocInternal->Sample_Number_To_Process,&max_power);
  power=max_power;
  for(i=SLocInternal->Sample_Number_To_Process;i<SLocInternal->Sample_Number_To_Store;i++)
  {
    power = power - SQR((q63_t)mic_tmp[i-SLocInternal->Sample_Number_To_Process]) + SQR((q63_t)mic_tmp[i]);
    if(power>max_power)
    {
      max_power=power;
      starting_index=(int32_t)i-(int32_t)SLocInternal->Sample_Number_To_Process;
    }
  }
  
  //DFT
  ComputeDFT(SLocInternal, SLocInternal->M1_Data, starting_index);
  
  //VAD
  float32_t energy_voice = 0.0f; 
  float32_t energy_noise;   
  
  SLocInternal->ptr_freq_energies=(float32_t*)USE_M1_MEM; //DFT_LEN/2
  
  arm_cmplx_mag_f32(SLocInternal->FFT_Out,SLocInternal->ptr_freq_energies,SLocInternal->Sample_Number_To_Process/2U);
  
  SLocInternal->ptr_freq_energies[0] = ABS((SLocInternal->FFT_Out[0]));
  SLocInternal->ptr_freq_energies[SLocInternal->Sample_Number_To_Process/2U] = ABS((SLocInternal->FFT_Out[1]));
  energy_noise = SLocInternal->ptr_freq_energies[0] + SLocInternal->ptr_freq_energies[SLocInternal->Sample_Number_To_Process/2U];
  
  for (i = 1; i < (SLocInternal->Sample_Number_To_Process / 2U); i++)
  {
    if (i < SLocInternal->freq_range_min) 
    {
      energy_noise += SLocInternal->ptr_freq_energies[i];
    }
    else
    {
      if(i < SLocInternal->freq_range_max) 
      {
        energy_voice += SLocInternal->ptr_freq_energies[i];
      }
      else
      {
        energy_noise += SLocInternal->ptr_freq_energies[i];  
      }
    }
  }  
  
  /**if (energy_voice / energy_noise > VAD_THRESHOLD) **/
  energy_noise*= (float32_t)SLocInternal->EVENT_THRESHOLD;
  /** default threshold GCCP 800 
  // default threshold BMPH 2.1 
  // [0,800,32768] -> [0,x,MAX_TH] 
  // 800:32768=x:MAX_TH x=800/32768*MAX_TH
  // [0,x] -> [0,2.1] 
  // x*y=2.1 y=2.1/(800/32768*MAX_TH) => y=86/MAX_TH **/
  energy_noise /= ((float32_t)MAX_THRESHOLD/86.0f) ; // rescaling compliant to GCCP algorithm
  if (energy_voice > energy_noise) 
  {
    ret = starting_index;
  }
  return ret;
}


/* dz: Compute the DFT of a single mic. */
static void ComputeDFT(libSoundSourceLoc_Handler_Internal * SLocInternal, void * M_Data, int32_t starting_index)
{
  UNUSED(starting_index);  
  arm_q15_to_float( &(((q15_t*)M_Data)[SLocInternal->mics_read_offset/*+starting_index*/])  , SLocInternal->mic_tmp , SLocInternal->Sample_Number_To_Process);
  arm_rfft_fast_f32(SLocInternal->SFast,SLocInternal->mic_tmp , SLocInternal->FFT_Out ,0);
}


/* dz: Dynamically select the most relevant frequencies. */
//compute the DFT of the microphones and select the frequencies corresponding to
//peaks in the spectum (w.r.t. only the first mic, for computational convenience).
static void DynamicFrequenciesSelection(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t starting_index)
{
  int16_t w;  
  uint16_t i;
  
  //recycle memory
  int16_t * freq_tmp =(int16_t*)&(SLocInternal->mic_tmp[0]); //NUM _OF_FREQ
  int8_t * freq_energies_maxima = (int8_t*)&(freq_tmp[SLocInternal->num_of_freq]); //DFT_LEN/2
  int16_t * perm_fr_en_max =(int16_t*)&(freq_energies_maxima[SLocInternal->Sample_Number_To_Process / 2U]); //DFT_LEN/2
  
  //uniformly spaced (except extrema)
  for (w=0; w<(int16_t)SLocInternal->num_of_freq; w++)
  {
    SLocInternal->frequencies_under_analysis[w] = (int16_t)SLocInternal->freq_range_min + ((w+1)*((int16_t)SLocInternal->freq_range_max-(int16_t)SLocInternal->freq_range_min)/((int16_t)SLocInternal->num_of_freq+1)) ;
    freq_tmp[w] = (int16_t)SLocInternal->freq_range_min + ((w+1)*((int16_t)SLocInternal->freq_range_max-(int16_t)SLocInternal->freq_range_min)/((int16_t)SLocInternal->num_of_freq+1) ) ;
  }
  
  //find local maxima in the interval of interest
  i=SLocInternal->freq_range_min+1U;
  while (i<SLocInternal->freq_range_max)
  {
    if ((SLocInternal->ptr_freq_energies[i]>=SLocInternal->ptr_freq_energies[i-1U]) && (SLocInternal->ptr_freq_energies[i]>=SLocInternal->ptr_freq_energies[i+1U]))
    {
      freq_energies_maxima[i] = 1;
      i++;
    }
    i++;
  }
  if ( SLocInternal->ptr_freq_energies[SLocInternal->freq_range_min]>= SLocInternal->ptr_freq_energies[SLocInternal->freq_range_min+1U])
  {
    freq_energies_maxima[SLocInternal->freq_range_min] = 1;
  }
  if ( SLocInternal->ptr_freq_energies[SLocInternal->freq_range_max]>= SLocInternal->ptr_freq_energies[SLocInternal->freq_range_max-1U])
  {
    freq_energies_maxima[SLocInternal->freq_range_max] = 1;
  }
  for (i=SLocInternal->freq_range_min+1U; i<SLocInternal->freq_range_max; i++)
  {
    if (freq_energies_maxima[i] != 1)
    {
      SLocInternal->ptr_freq_energies[i] = 0.0f;
    }
  }  
  
  //sort energies in descending order and store them
  sort_desc_f32(SLocInternal->ptr_freq_energies,(int32_t)SLocInternal->Sample_Number_To_Process/2,perm_fr_en_max);
  
  //merge maxima with predefined frequencies
  int32_t min_i,min,tmp;
  (void)memset(freq_energies_maxima,0,SLocInternal->Sample_Number_To_Process/2U*sizeof(int8_t));
  
  for(w=0; w<(int32_t)SLocInternal->num_of_freq; w++)
  {
    if (SLocInternal->ptr_freq_energies[perm_fr_en_max[w]]<=0.0f)
    {
      break;
    }
    else
    {
      min=(int32_t)SLocInternal->Sample_Number_To_Process;
      min_i=-1;
      for(i=0;i<SLocInternal->num_of_freq;i++)
      {
        tmp=(int32_t)freq_tmp[i]-(int32_t)perm_fr_en_max[w];
        tmp=(int32_t)ABS((tmp));
        if (tmp<min)
        {
          min=tmp;
          min_i=(int32_t)i;
        }
      }
      SLocInternal->frequencies_under_analysis[min_i]=perm_fr_en_max[w];
      freq_tmp[min_i]=-((int16_t)SLocInternal->Sample_Number_To_Process);
    }
  }
  
  //sort frequencies in ascending index
  sort_desc_i16(SLocInternal->frequencies_under_analysis,(int32_t)SLocInternal->num_of_freq,perm_fr_en_max);
  int16_t tmp2;
  for(w=0; w<((int16_t)(SLocInternal->num_of_freq)/2); w++)
  {
    tmp2= SLocInternal->frequencies_under_analysis[ perm_fr_en_max[w]];
    SLocInternal->frequencies_under_analysis[perm_fr_en_max[w]] = SLocInternal->frequencies_under_analysis[perm_fr_en_max[SLocInternal->num_of_freq-1U-(uint16_t)w]];
    SLocInternal->frequencies_under_analysis[perm_fr_en_max[SLocInternal->num_of_freq-1U-(uint16_t)w]] = tmp2;
  }
  
  //store freq of interest
  ExtractFrequenciesOfInterest(SLocInternal,starting_index);
}



/* dz: Return the permutation that sorts the input in descending order. */
//a[p[.]] is sorted; implement the insertion sort algorithm.
static void sort_desc_f32(float32_t *a, int32_t len, int16_t *p)
{  
  int16_t i,j,tmpp;
  
  //init
  for (i=0;i<len;i++)
  {
    p[i]=i;
  }
  
  //insertion sort
  for (i=1;i<len;i++)
  {
    j = i;
    while ( (j > 0) && ((float32_t)(a[p[j-1]]) < (float32_t)(a[p[j]])) )
    {
      tmpp=p[j]; 
      p[j]=p[j-1];
      p[j-1]=tmpp;
      j--;
    }
  }
}

static void sort_desc_i16(int16_t *a, int32_t len, int16_t *p)
{
  int16_t i,j,tmpp;
  
  //init
  for (i=0;i<len;i++)
  {
    p[i]=i;
  }
  
  //insertion sort
  for (i=1;i<len;i++)
  {
    j = i;
    while ( (j > 0) && ((int16_t)(a[p[j-1]]) < (int16_t)(a[p[j]])) )
    {
      tmpp=p[j]; 
      p[j]=p[j-1];
      p[j-1]=tmpp;
      j--;
    }
  }
}


/* dz: Extract the Frequencies Of Interest frome the whole DFT. */
static void ExtractFrequenciesOfInterest(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t starting_index)
{
  uint16_t m,w;
  
  for (m=0;m<SLocInternal->Mic_Number;m++)
  {    
    //compute DFT (first mic already processed in VAD)
    if(m==1U)
    {
      ComputeDFT(SLocInternal,SLocInternal->M2_Data, starting_index);
    }
    else if(m==2U)
    {
      ComputeDFT(SLocInternal,SLocInternal->M3_Data, starting_index);
    }
    else if(m==3U)
    {
      ComputeDFT(SLocInternal,SLocInternal->M4_Data, starting_index);
    } 
    else
    {
      /* no other use cases are handled */
    }
    
    //extract only the frequency of interest
    for (w=0;w<SLocInternal->num_of_freq;w++)
    {      
      //store the frequency of interest
      if(SLocInternal->frequencies_under_analysis[w]==0)
      {
        SLocInternal->s[(w*2U*SLocInternal->Mic_Number) + (2U*(uint16_t)SLocInternal->remap_mic[m]) + 0U]=SLocInternal->FFT_Out[0];
        SLocInternal->s[(w*2U*SLocInternal->Mic_Number) + (2U*(uint16_t)SLocInternal->remap_mic[m]) + 1U]=0.0f;
      }
      else if(SLocInternal->frequencies_under_analysis[w] == ((int16_t)SLocInternal->Sample_Number_To_Process/2))
      {
        SLocInternal->s[(w*2U*SLocInternal->Mic_Number) + (2U*(uint16_t)SLocInternal->remap_mic[m]) + 0U]=SLocInternal->FFT_Out[1];
        SLocInternal->s[(w*2U*SLocInternal->Mic_Number) + (2U*(uint16_t)SLocInternal->remap_mic[m]) + 1U]=0.0f;
      }
      else
      {
        SLocInternal->s[(w*2U*SLocInternal->Mic_Number) + (2U*(uint16_t)SLocInternal->remap_mic[m]) + 0U]=SLocInternal->FFT_Out[(2*SLocInternal->frequencies_under_analysis[w]) + 0];
        SLocInternal->s[(w*2U*SLocInternal->Mic_Number) + (2U*(uint16_t)SLocInternal->remap_mic[m]) + 1U]=SLocInternal->FFT_Out[(2*SLocInternal->frequencies_under_analysis[w]) + 1];
      }      
    }    
  }
}


/* Aggregate the estimated DoAs for a more stable response. */
static int16_t StableDoA(libSoundSourceLoc_Handler_Internal * SLocInternal, int32_t i_max, float32_t v_max)
{  
  int16_t ret = -100;
  
  //recycle memory
  int16_t *perm=(int16_t*)USE_M2_MEM; 
  uint16_t n,ct; 
  
  //new
  if (SLocInternal->resolution<=15U)
  {
    SLocInternal->local_stabilizer=1;
  }
  else
  {
    SLocInternal->local_stabilizer=0;
  }
  
  if (SLocInternal->reactivity==0U)
  {  
    /* punctual  */
    //sources get back to its original use: store the angles of arrival in 
    //descending order of magnitude [ang, mag, ang, mag]
    (void)memset(SLocInternal->score_theta,0,SLocInternal->num_of_angles*sizeof(int16_t));
    SLocInternal->score_theta[i_max]=(int16_t)((1000*(int32_t)v_max));
  }
  else
  { 
    /* averaged */
    for (n=0;n<SLocInternal->num_of_angles;n++)
    {
      if ( (n==(uint16_t)i_max) && (v_max>0.0f) )
      {
        SLocInternal->score_theta[(n+SLocInternal->num_of_angles)%SLocInternal->num_of_angles] = MIN((int16_t)SLocInternal->stability, SLocInternal->score_theta[(n+SLocInternal->num_of_angles)%SLocInternal->num_of_angles] + (int16_t)SLocInternal->reactivity);
        if (SLocInternal->local_stabilizer == 1U)
        {
          SLocInternal->score_theta[(n-1U+SLocInternal->num_of_angles)%SLocInternal->num_of_angles] = MIN((int16_t)SLocInternal->stability , SLocInternal->score_theta[(n-1U+SLocInternal->num_of_angles)%SLocInternal->num_of_angles] + ((int16_t)SLocInternal->reactivity/3));
          SLocInternal->score_theta[(n+1U+SLocInternal->num_of_angles)%SLocInternal->num_of_angles] = MIN((int16_t)SLocInternal->stability , SLocInternal->score_theta[(n+1U+SLocInternal->num_of_angles)%SLocInternal->num_of_angles] + ((int16_t)SLocInternal->reactivity/3));
        }
      }
      else
      {
        if (SLocInternal->score_theta[n]>0)
        {
          SLocInternal->score_theta[n]--;
        }
      }      
    }    
  }
  
  //reset
  arm_fill_q15(-500,SLocInternal->sources,2U*NUM_OUTPUT_SOURCES);
  
  sort_desc_i16(SLocInternal->score_theta, (int32_t)SLocInternal->num_of_angles, perm);
  
  for(ct=0;ct<NUM_OUTPUT_SOURCES;ct++)
  {
    if (SLocInternal->score_theta[perm[ct]]>0)
    {      
      SLocInternal->sources[(2U*ct)+0U]=adjust_output_angle(SLocInternal->theta[perm[ct]],SLocInternal->array_type);
      SLocInternal->sources[(2U*ct)+1U]=SLocInternal->score_theta[perm[ct]]; 
    }
  }  
  
  if (SLocInternal->sources[1] > ((int16_t)SLocInternal->reactivity*4/3) )
  {
    ret = SLocInternal->sources[0];
  }
  return ret;
}


static int16_t adjust_output_angle(float32_t theta, uint8_t array)
{  
  if (array==LINEAR_ARRAY)
  {
    theta=((theta-PI_1_6)*1.5f) - PI_1_2;
  }
  else if (array==CIRCULAR_ARRAY)
  {
    theta = - theta - PI_3_2;
    while(theta>PI_2)
    {
      theta -= PI_2;
    }
    while(theta<0.0f)
    {
      theta += PI_2;
    }
  }
  else
  {
    /* No other use cases are handled */
  }
  
  float32_t tmp = theta * FACTOR_RAD_2_DEG;
  return (int16_t)(tmp);  
}


//volatile float32_t test_en[MAX_NUM_OF_ANGLES];
static void max_block_correlation(libSoundSourceLoc_Handler_Internal * SLocInternal,int32_t *i_max, float32_t *v_max)
{
  //recycle memory
  float32_t *steering_vec = (float32_t*)USE_M2_MEM; // 2* AUDIO_CHANNELS
  float32_t *om = &(steering_vec[2U*SLocInternal->Mic_Number]);  // NUM_OF_FREQ
  
  if ((SLocInternal->num_of_angles + (2U*SLocInternal->Mic_Number) + SLocInternal->num_of_freq) <= (uint16_t)SLocInternal->Sample_Number_To_Store)
  {    
    float32_t arg;
    float32_t dp_i,dp_r;
    float32_t en_n;
    
    int32_t n,m,w;
    
    for (w=0;w<(int32_t)SLocInternal->num_of_freq;w++)
    {
      om[w]= ((float32_t)SLocInternal->frequencies_under_analysis[w]) * (float32_t)FACTOR_INDEX_2_HZ;
    }
    
    *v_max=0.0f;
    *i_max=-1;
    
    for (n=0;n<(int32_t)SLocInternal->num_of_angles;n++)
    {
      en_n=0.0f;
      for (w=0;w<(int32_t)SLocInternal->num_of_freq;w++)
      {
        for (m=0;m<(int32_t)SLocInternal->Mic_Number;m++)
        {        
          arg= SLocInternal->steering_tau[(n*(int32_t)SLocInternal->Mic_Number)+m];
          arg *= -2.0f;
          arg *= PI;
          arg *= om[w];
          dz_cos_msin_f32_table(arg,&(steering_vec[2*m]));
        }
        
        arm_cmplx_dot_prod_f32 (&(SLocInternal->s[w*2*(int32_t)SLocInternal->Mic_Number]), steering_vec, SLocInternal->Mic_Number, &dp_r, &dp_i);
        en_n += SQR(dp_r) + SQR(dp_i);
      }
      if (en_n>*v_max)
      {
        *v_max=en_n;
        *i_max=n;
      }
    }
  }
}


static float32_t dz_sin_f32(float32_t x)
{  
  float32_t y;
  int32_t top=0;
  
  while(x<0.0f)
  {
    x=x+(PI_2);
  }
  while(x>PI_2)
  {
    x=x-(PI_2);
  }
  if (x>PI)
  {
    top=-1;
    x=-x+PI_2;
  }  
  if (x>PI_1_2)
  {
    x=-x+PI;
  }
  if(x<=PI_1_6)
  {
    y=0.954929658551372f * x;
  }
  else
  {
    y=( ((-0.43f * x) + 1.378054723304760f) * x) - 0.103660824372369f;
  }
  if (top==-1)
  {
    y=-y;
  }
  return y;
}


static float32_t dz_cos_f32(float32_t x)
{
  return dz_sin_f32(x+(PI/2.0f));
}

static void dz_cos_msin_f32_table(float32_t x,float32_t* s)
{
  float32_t dz_sin_f32table[90]={0.00873f, 0.02618f, 0.04362f, 0.06105f, 0.07846f, 
  0.09585f, 0.11320f, 0.13053f, 0.14781f, 0.16505f, 0.18224f, 0.19937f, 0.21644f, 0.23345f, 0.25038f, 
  0.26724f, 0.28402f, 0.30071f, 0.31730f, 0.33381f, 0.35021f, 0.36650f, 0.38268f, 0.39875f, 0.41469f, 
  0.43051f, 0.44620f, 0.46175f, 0.47716f, 0.49242f, 0.50754f, 0.52250f, 0.53730f, 0.55194f, 0.56641f, 
  0.58070f, 0.59482f, 0.60876f, 0.62251f, 0.63608f, 0.64945f, 0.66262f, 0.67559f, 0.68835f, 0.70091f, 
  0.71325f, 0.72537f, 0.73728f, 0.74896f, 0.76041f, 0.77162f, 0.78261f, 0.79335f, 0.80386f, 0.81412f, 
  0.82413f, 0.83389f, 0.84339f, 0.85264f, 0.86163f, 0.87036f, 0.87882f, 0.88701f, 0.89493f, 0.90259f, 
  0.90996f, 0.91706f, 0.92388f, 0.93042f, 0.93667f, 0.94264f, 0.94832f, 0.95372f, 0.95882f, 0.96363f, 
  0.96815f, 0.97237f, 0.97630f, 0.97992f, 0.98325f, 0.98629f, 0.98902f, 0.99144f, 0.99357f, 0.99540f, 
  0.99692f, 0.99813f, 0.99905f, 0.99966f, 0.99996f};

  float32_t index_f;
  int16_t index; 
  index_f = x*FACTOR_RAD_2_DEG;
  index = (int16_t)index_f;
  
  while(index<0)
  {
    index=index+360;
  }
  while(index>360)
  {
    index=index-360;
  }
  
  uint8_t top=1,right=1;
  
  if (index>=180)
  {
    top=0;
  }
  if (index<270)
  {
    if (index>=90)
    {
      right=0;
    }
  }
    
  if (top == 1U)
  {
    if (right == 1U)
    {
      s[1]= dz_sin_f32table[index];
      s[0]=-dz_sin_f32table[89-index];
    }
    else
    {
      index=179-index; 
      s[1]= dz_sin_f32table[index];
      s[0]= dz_sin_f32table[89-index];
    }
  }
  else
  {
    if (right == 1U)
    {
      index=359-index; 
      s[1]=-dz_sin_f32table[index];
      s[0]=-dz_sin_f32table[89-index];
    }
    else
    {
      index=index-180;
      s[1]=-dz_sin_f32table[index];
      s[0]= dz_sin_f32table[89-index];
    }
  }
}

#endif /*__DOA_VIA_BLOCK_SPARSITY_H*/

