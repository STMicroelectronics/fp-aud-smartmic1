/**
******************************************************************************
* @file    defines.h
* @author  SRA
* @brief   This file contains Acoustic Beamforming library definitions.
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
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEFINES_H
#define __DEFINES_H

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "math.h"
#include "pdm2pcm_glo.h"

/* Exported types ------------------------------------------------------------*/
typedef int16_t spx_int16_t;
typedef uint16_t spx_uint16_t;
typedef int32_t spx_int32_t;
typedef uint32_t spx_uint32_t;

typedef float32_t spx_mem_t;
typedef float32_t spx_coef_t;
typedef float32_t spx_lsp_t;
typedef float32_t spx_sig_t;
typedef float32_t spx_word16_t;
typedef float32_t spx_word32_t;
typedef float32_t spx_float_t;

/* Exported constants --------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define N_MIC_MAX 1
#define N_SPEKAER_MAX 1
#define NN_MAX 128
#define TAIL_MAX 1
#define NB_BANDS 24
#define ECHO_BUFF 128U
#define SPEEX_ECHO_SET_SAMPLING_RATE 24
#define SPEEX_PREPROCESS_SET_ECHO_STATE 24
#define TWO_PATH

#define NN 128
#define TAIL 1
#define INTERNAL_BUFF_SIZE                256U
#define GAIN_COMPUTATION_LENGTH 8000U
#define STANDARD_IIR

#define Q15ONE 1.0f
#define LPC_SCALING  1.f
#define SIG_SCALING  1.f
#define LSP_SCALING  1.f
#define GAMMA_SCALING 1.f
#define GAIN_SCALING 1.f
#define GAIN_SCALING_1 1.f

#define VERY_SMALL 1e-15f
#define VERY_LARGE32 1e15f
#define VERY_LARGE16 1e15f
#define Q15_ONE ((spx_word16_t)1.f)

#define FLOAT_ZERO 0.f
#define FLOAT_ONE 1.f
#define FLOAT_HALF 0.5f

/* Exported macro ------------------------------------------------------------*/
#define SaturaL(N, L) (((N)<(L))?(L):(N))
#define SaturaH(N, H) (((N)>(H))?(H):(N))
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define ABS(x) ( ((x) < 0) ? (-(x)) : (x))      /**< Absolute integer value. */
#define ABS16(x) ( ((x) < 0) ? (-(x)) : (x))    /**< Absolute 16-bit value.  */
#define MIN16(a,b) ( ((a) < (b)) ? (a) : (b))   /**< Maximum 16-bit value.   */
#define MAX16(a,b) ( ((a) > (b)) ? (a) : (b))   /**< Maximum 16-bit value.   */
#define ABS32(x) ( (((float32_t)x) < 0.0f) ? (-((float32_t)x)) : ((float32_t)x))    /**< Absolute 32-bit value.  */
#define MIN32(a,b) ( (((float32_t)a) < ((float32_t)b)) ? ((float32_t)a) : ((float32_t)b))   /**< Maximum 32-bit value.   */
#define MAX32(a,b) ( (((float32_t)a) > ((float32_t)b)) ? ((float32_t)a) : ((float32_t)b))   /**< Maximum 32-bit value.   */

#ifndef UNUSED
#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#endif

#define QCONST16(x,bits) (x)
#define QCONST32(x,bits) (x)

#define NEG16(x) (-(x))
#define NEG32(x) (-((float32_t)x))
#define EXTRACT16(x) (x)
#define EXTEND32(x) ((float32_t)x)
#define SHR16(a,shift) (a)
#define SHL16(a,shift) (a)
#define SHR32(a,shift) ((float32_t)a)
#define SHL32(a,shift) ((float32_t)a)
#define PSHR16(a,shift) (a)
#define PSHR32(a,shift) ((float32_t)a)
#define VSHR32(a,shift) ((float32_t)a)
#define SATURATE16(x,a) (x)
#define SATURATE32(x,a) ((float32_t)x)

#define PSHR(a,shift)       (a)
#define SHR(a,shift)       (a)
#define SHL(a,shift)       (a)
#define SATURATE(x,a) (x)

#define ADD16(a,b) ((a)+(b))
#define SUB16(a,b) ((a)-(b))
#define ADD32(a,b) ((a)+(b))
#define SUB32(a,b) ((a)-(b))
#define MULT16_16_16(a,b)     ((a)*(b))
#define MULT16_16(a,b)     ((spx_word32_t)(a)*(spx_word32_t)(b))
#define MAC16_16(c,a,b)     ((c)+ ((spx_word32_t)(a)*(spx_word32_t)(b)) )

#define MULT16_32_Q11(a,b)     ((a)*(b))
#define MULT16_32_Q13(a,b)     ((a)*(b))
#define MULT16_32_Q14(a,b)     ((a)*(b))
#define MULT16_32_Q15(a,b)     ((a)*(b))
#define MULT16_32_P15(a,b)     ((a)*(b))

#define MAC16_32_Q11(c,a,b)     ((c)+(a)*(b))
#define MAC16_32_Q15(c,a,b)     ((c)+(a)*(b))

#define MAC16_16_Q11(c,a,b)     ((c)+(a)*(b))
#define MAC16_16_Q13(c,a,b)     ((c)+(a)*(b))
#define MAC16_16_P13(c,a,b)     ((c)+(a)*(b))
#define MULT16_16_Q11_32(a,b)     ((a)*(b))
#define MULT16_16_Q13(a,b)     ((a)*(b))
#define MULT16_16_Q14(a,b)     ((a)*(b))
#define MULT16_16_Q15(a,b)     ((a)*(b))
#define MULT16_16_P15(a,b)     ((a)*(b))
#define MULT16_16_P13(a,b)     ((a)*(b))
#define MULT16_16_P14(a,b)     ((a)*(b))

#define DIV32_16(a,b)     (((spx_word32_t)(a))/(spx_word16_t)(b))
#define PDIV32_16(a,b)     (((spx_word32_t)(a))/(spx_word16_t)(b))
#define DIV32(a,b)     (((spx_word32_t)(a))/(spx_word32_t)(b))
#define PDIV32(a,b)     (((spx_word32_t)(a))/(spx_word32_t)(b))

#define PSEUDOFLOAT(x) (x)
#define FLOAT_MULT(a,b) ((a)*(b))
#define FLOAT_AMULT(a,b) ((a)*(b))
#define FLOAT_MUL32(a,b) ((a)*(b))
#define FLOAT_DIV32(a,b) ((a)/(b))
#define FLOAT_EXTRACT16(a) (a)
#define FLOAT_EXTRACT32(a) (a)
#define FLOAT_ADD(a,b) ((a)+(b))
#define FLOAT_SUB(a,b) ((a)-(b))
#define REALFLOAT(x) (x)
#define FLOAT_DIV32_FLOAT(a,b) ((a)/(b))
#define FLOAT_MUL32U(a,b) ((a)*(b))
#define FLOAT_SHL(a,b) (a)
#define FLOAT_LT(a,b) ((a)<(b))
#define FLOAT_GT(a,b) ((a)>(b))
#define FLOAT_DIVU(a,b) ((a)/(b))
#define FLOAT_SQRT(a) (spx_sqrt(a))

/* Exported define -----------------------------------------------------------*/
#define spx_sqrt my_sqrt  //sqrt
#define spx_acos acosf
#define spx_exp expf
#define spx_cos_norm(x) (cosf((.5f*M_PI)*(x)))
#define spx_atan atanf

static inline float32_t my_sqrt(float32_t a)
{
  float32_t b;
  (void)arm_sqrt_f32 (a, &b);
  return b;
}

/* External variables --------------------------------------------------------*/
typedef struct {
  uint16_t jump_in;
  uint16_t counter_out;
  uint16_t len;
  uint16_t delay;
  uint8_t last_part[16];
  uint8_t nBytes;
  uint8_t nBits;
  uint64_t last_part_64;
} TDelay;

typedef struct {  
  int32_t OldOut, OldIn, OldZ;
  uint8_t BF;
  int32_t y,y_old,x,x_old;
  int32_t s1_out_old,s2_out_old;
  uint8_t HP;
  uint16_t HP_ALFA;
  uint16_t OldOut_s1;
  uint16_t OldIn_s1;
  uint16_t OldOut_s2;
  uint16_t OldIn_s2;
  uint16_t alpha_antifilter;
  uint16_t gain_antifilter_s1;
  uint16_t gain_antifilter_s2;
  uint8_t MicChannels;
} TPCMBF;

typedef uint16_t pcm_sample;
typedef uint8_t pdm_sample;

/** Discrete Rotational Fourier Transform lookup */
typedef struct{
  int32_t n;
  float32_t trigcache[INTERNAL_BUFF_SIZE*3];
  int32_t splitcache[32];
}drft_lookup;

typedef struct {
  int32_t  bank_left[NN_MAX];
  int32_t   bank_right[NN_MAX];
  spx_word16_t  filter_left[NN_MAX];
  spx_word16_t  filter_right[NN_MAX];
#ifndef FIXED_POINT
  float32_t scaling[NB_BANDS];
#endif
  int32_t nb_banks;
  int32_t len;
} FilterBank;

typedef struct   {
  int32_t frame_size;           /**< Number of samples processed each time */
  int32_t window_size;
  int32_t M;
  int32_t cancel_count;
  int32_t adapted;
  int32_t saturated;
  int32_t screwed_up;
  int32_t C;                    /** Number of input channels (microphones) */
  int32_t K;                    /** Number of output channels (loudspeakers) */
  spx_int32_t sampling_rate;
  spx_word16_t spec_average;
  spx_word16_t beta0;
  spx_word16_t beta_max;
  spx_word32_t sum_adapt;
  spx_word16_t leak_estimate;
  spx_word16_t e[N_MIC_MAX*NN_MAX*2];
  spx_word16_t x[N_SPEKAER_MAX*NN_MAX*2];       /* Far-end input buffer (2N): x */
  spx_word16_t X[N_SPEKAER_MAX*(((TAIL_MAX+NN_MAX-1)/NN_MAX)+1)*NN_MAX*2];      /* Far-end buffer (M+1 frames) in frequency domain: X */
  spx_word16_t input[N_MIC_MAX*NN_MAX];         /* scratch: input */
  spx_word16_t y[N_MIC_MAX*NN_MAX*2];           /* scratch: y */
  spx_word16_t last_y[N_MIC_MAX*NN_MAX*2];      /* last_y */
  spx_word16_t Y[N_MIC_MAX*NN_MAX*2];           /* scratch: Y */
  spx_word16_t E[N_MIC_MAX*NN_MAX*2];           /* E */
  spx_word32_t PHI[NN_MAX*2];                   /* scratch: PHI */
  spx_word32_t W[N_MIC_MAX*N_SPEKAER_MAX*((TAIL_MAX+NN_MAX-1)/NN_MAX)*NN_MAX*2];/* (Background) filter weights: W */
#ifdef TWO_PATH
  spx_word16_t foreground[N_MIC_MAX*N_SPEKAER_MAX*((TAIL_MAX+NN_MAX-1)/NN_MAX)*NN_MAX*2];       /* Foreground filter weights: foreground */
  spx_word32_t  Davg1;                          /* 1st recursive average of the residual power difference */
  spx_word32_t  Davg2;                          /* 2nd recursive average of the residual power difference */
  spx_float_t   Dvar1;                          /* Estimated variance of 1st estimator */
  spx_float_t   Dvar2;                          /* Estimated variance of 2nd estimator */
#endif
  spx_word32_t power[NN_MAX+1];                 /* Power of the far-end signal: power */
  spx_float_t  power_1[NN_MAX+1];               /* Inverse power of far-end: power_1 */
  spx_word16_t wtmp[NN_MAX];                    /*wtmp */
  spx_word32_t Rf[(NN_MAX+1)];                  /*Rf */
  spx_word32_t Yf[(NN_MAX+1)];                  /*Yf */
  spx_word32_t Xf[(NN_MAX+1)];                  /*Xf */
  spx_word32_t Eh[(NN_MAX+1)];                  /*Eh */
  spx_word32_t Yh[(NN_MAX+1)];                  /*Yh */
  spx_float_t   Pey;
  spx_float_t   Pyy;
  spx_word16_t window[NN_MAX*2];                /*window */ 
  spx_word16_t prop[(TAIL_MAX)];                /*prop */
  drft_lookup *fft_table;
  spx_word16_t memX;
  spx_word16_t memD;
  spx_word16_t memE;
  spx_word16_t preemph;
  spx_word16_t notch_radius;
  spx_mem_t notch_mem[2*N_MIC_MAX];             /*notch_mem */
  
  spx_int16_t play_buf[1];
  int32_t play_buf_pos;
  int32_t play_buf_started;
}SpeexEchoState;

/** Speex pre-processor state. */
typedef struct {
  /* Basic info */
  int32_t    frame_size;                            /**< Number of samples processed each time */
  int32_t    ps_size;                               /**< Number of points in the power spectrum */
  int32_t    sampling_rate;                         /**< Sampling rate of the input/output */
  int32_t    nbands;
  FilterBank *bank;
  
  /* Parameters */
  int32_t    denoise_enabled;
  int32_t    vad_enabled;
  int32_t    dereverb_enabled;
  spx_word16_t  reverb_decay;
  spx_word16_t  reverb_level;
  spx_word16_t speech_prob_start;
  spx_word16_t speech_prob_continue;
  int32_t    noise_suppress;
  int32_t    echo_suppress;
  int32_t    echo_suppress_active;
  SpeexEchoState *echo_state;
  
  spx_word16_t	speech_prob;                    /**< Probability last frame was speech */
  
  /* DSP-related arrays */
  spx_word16_t frame[2*NN_MAX];                 /**< Processing frame (2*ps_size) */
  spx_word16_t ft[2*NN_MAX];                    /**< Processing frame in freq domain (2*ps_size) */
  spx_word32_t ps[NN_MAX+NB_BANDS];             /**< Current power spectrum */
  spx_word16_t gain2[NN_MAX+NB_BANDS];          /**< Adjusted gains */
  spx_word16_t gain_floor[NN_MAX+NB_BANDS];     /**< Minimum gain allowed */
  float32_t window[2*NN_MAX];
  spx_word32_t noise[NN_MAX+NB_BANDS];          /**< Noise estimate */
  spx_word32_t old_ps[NN_MAX+NB_BANDS];         /**< Power spectrum for last frame */
  spx_word16_t gain[NN_MAX+NB_BANDS];           /**< Ephraim Malah gain */
  spx_word16_t prior[NN_MAX+NB_BANDS];          /**< A-priori SNR */
  spx_word16_t post[NN_MAX+NB_BANDS];           /**< A-posteriori SNR */
  
  spx_word32_t S[NN_MAX];                       /**< Smoothed power spectrum */
  spx_word32_t Smin[NN_MAX];                    /**< See Cohen paper */
  spx_word32_t Stmp[NN_MAX];                    /**< See Cohen paper */
  int32_t update_prob[NN_MAX];                      /**< Probability of speech presence for noise update */
  
  spx_word16_t zeta[NN_MAX+NB_BANDS];           /**< Smoothed a priori SNR */
  spx_word32_t echo_noise[NN_MAX+NB_BANDS];
  spx_word32_t residual_echo[NN_MAX+NB_BANDS];
  
  /* Misc */
  spx_word16_t inbuf[NN_MAX];                   /**< Input buffer (overlapped analysis) */
  spx_word16_t outbuf[NN_MAX];                  /**< Output buffer (for overlap and add) */
  
  /* AGC stuff, only for floating point for now */
#ifndef FIXED_POINT
  int32_t    agc_enabled;
  float32_t  agc_level;
  float32_t  loudness_accum;
  float32_t  loudness;          /**< Loudness estimate */
  float32_t  agc_gain;          /**< Current AGC gain */
  float32_t  max_gain;          /**< Maximum gain allowed */
  float32_t  max_increase_step; /**< Maximum increase in gain from one frame to another */
  float32_t  max_decrease_step; /**< Maximum decrease in gain from one frame to another */
  float32_t  prev_loudness;     /**< Loudness of previous frame */
  float32_t  init_max;          /**< Current gain limit during initialisation */
#endif
  int32_t    nb_adapt;          /**< Number of frames used for adaptation so far */
  int32_t    was_speech;
  int32_t    min_count;         /**< Number of frames processed so far */
  drft_lookup  *fft_lookup;        /**< Lookup table for the FFT */
#ifdef FIXED_POINT
  int32_t    frame_shift;
#endif
}SpeexPreprocessState;


/*DATA STRUCTURES*/
struct _libBeamforming_Handler_Internal;

typedef struct {
  uint32_t  (*PDM_Filter_Standard_M1)(void *pDataIn, void *pDataOut, PDM_Filter_Handler_t *pHandler);
  uint32_t  (*PDM_Filter_Standard_M2)(void *pDataIn, void *pDataOut, PDM_Filter_Handler_t *pHandler);
  uint32_t  (*PDM_Filter_Delayed)(void *pDataIn, void *pDataOut, PDM_Filter_Handler_t *pHandler);  
  void (*Delay)(void *buffer_in, struct _libBeamforming_Handler_Internal * BeamFormingInternal, TDelay *Delay_struct);   
  uint8_t (*FirstStep)(void *pM1, void *pM2, int16_t *ptr_Out, struct _libBeamforming_Handler_Internal * BeamFormingInternal);  
}libBeamforming_Handler_Callbacks;

typedef struct _libBeamforming_Handler_Internal{  
  uint32_t algorithm_type_init;  //
  uint32_t sampling_frequency;  //
  uint32_t data_format;  //
  uint32_t Buffer_State;  //
  uint8_t ptr_M1_channels;  //
  uint8_t ptr_M2_channels;  //
  uint16_t mic_distance; //
  float32_t M2_gain; //
  uint16_t Samples_Count; //
  uint16_t Samples_Count_Output; //
  uint16_t initialized_as;  
  uint8_t counter;
  uint8_t delay_enable;
  uint32_t GainSamplesCounter_0;   
  float32_t RMSm1_0;
  float32_t RMSm2_0;
  float32_t gain_0;
  float32_t gain_old_0;
  uint8_t overall_gain;
  uint32_t ptr_out_channels;
  uint32_t ref_mic_enable;  
  libBeamforming_Handler_Callbacks Callbacks;
  uint8_t PDM_Delay_Buffer[256*2];
  uint16_t  OutBuff [16 * 4];   
  /***********/
  uint16_t  OutBuff_Beam [16 * 2]; 
#ifndef STANDARD_IIR
  q15_t biquadState1[8];//
  q15_t biquadState2[8];//
  q15_t coefficients15_1[12];//
  q15_t coefficients15_2[12];//
#endif
  int16_t * Dir1_Buffer; //
  int16_t * Dir2_Buffer; //
  int16_t * E_Buffer; //
  SpeexPreprocessState * den;//
  SpeexEchoState * st;//
  FilterBank * filterBank;//
  drft_lookup * table;//
  drft_lookup * table_den;//
  PDM_Filter_Handler_t * Filter_M1_Standard_Handler;//
  PDM_Filter_Handler_t * Filter_M2_Standard_Handler;//
  PDM_Filter_Handler_t * Filter_M1_Delayed_Handler;//
  PDM_Filter_Handler_t * Filter_M2_Delayed_Handler;//
  PDM_Filter_Config_t * Filter_M1_Standard_Config;//
  PDM_Filter_Config_t * Filter_M2_Standard_Config;//
  PDM_Filter_Config_t * Filter_M1_Delayed_Config;//
  PDM_Filter_Config_t * Filter_M2_Delayed_Config;//
  TDelay * Delay_M1;//
  TDelay * Delay_M2;//
  TPCMBF * BFParam_1;//
  TPCMBF * BFParam_2;//
  
  int32_t PostFilterEn;
#ifndef STANDARD_IIR
  arm_biquad_casd_df1_inst_q15 * S1;//
  arm_biquad_casd_df1_inst_q15 * S2;//  
#endif
} libBeamforming_Handler_Internal;


/* Exported functions ------------------------------------------------------- */

/*ADAPTIVE FILTER related*/
static void adaptiveget_residual(SpeexEchoState *st, spx_word32_t *residual_echo, int32_t len);
static void adaptive_A_run(SpeexEchoState *st, const spx_int16_t *in, const spx_int16_t *far_end, spx_int16_t *out);
static void adaptivestate_reset(SpeexEchoState *st);
static void adaptivestate_init_mc(SpeexEchoState *st,int32_t frame_size, int32_t filter_length, int32_t nb_mic, int32_t nb_speakers);

/*FFT WRAPPER related*/
static void fft(drft_lookup *table, float32_t *in, float32_t *out);
static void ifft(drft_lookup *table, float32_t *in, float32_t *out);
static void fft_init(drft_lookup *table, int32_t size);

/*FILTER BANK related*/
static void filterbank_new(FilterBank *bank, int32_t banks, spx_word32_t sampling, int32_t len, int32_t type);
static void filterbank_compute_psd16(FilterBank *bank, spx_word16_t *mel, spx_word16_t *ps);
static void filterbank_compute_bank32(FilterBank *bank, spx_word32_t *ps, spx_word32_t *mel);

/*DENOISE related*/
static int32_t denoiser_setup(SpeexPreprocessState *state, int32_t request, SpeexEchoState *ptr);
static int32_t denoiser_A_run(SpeexPreprocessState *st, spx_int16_t *x);
static void update_noise_prob(SpeexPreprocessState *st);
static void denoiser_analize(SpeexPreprocessState *st, spx_int16_t *x);
static void denoiserstate_init(SpeexPreprocessState * st,int32_t frame_size, int32_t sampling_rate);
static void compute_gain_floor(float32_t noise_suppress, float32_t effective_echo_suppress, spx_word32_t *noise, spx_word32_t *echo, spx_word16_t *gain_floor, int32_t len);
static inline spx_word32_t hypergeom_gain(spx_word32_t xx);

/*SMALLFFT related*/
static void drft_init(drft_lookup *l,int32_t n);
static void drft_backward(drft_lookup *l,float32_t *data);
static void drft_forward(drft_lookup *l,float32_t *data);

/*DELAY related*/
static void Delay_3(void * buffer_in, libBeamforming_Handler_Internal * BeamFormingInternal ,TDelay *Delay_struct);
static void Delay_2(void * buffer_in, libBeamforming_Handler_Internal * BeamFormingInternal ,TDelay *Delay_struct);
static void Delay_1(void * buffer_in, libBeamforming_Handler_Internal * BeamFormingInternal ,TDelay *Delay_struct);
static void Delay_Init(TDelay *Delay_struct) ;

/*ARM*/
arm_status arm_mat_mult_f32(  const arm_matrix_instance_f32 * pSrcA,  const arm_matrix_instance_f32 * pSrcB,  arm_matrix_instance_f32 * pDst);

/*DMA and ANTIFILTER related*/
static void BF_Init(TPCMBF *ParamBf, uint16_t Fs,uint8_t MicChannels, float32_t alpha_antifilter, float32_t gain_antifilter_s1 ,float32_t gain_antifilter_s2 );
#ifdef STANDARD_IIR
/*ALPHA;GAIN*/

static const float32_t coefficients_3mm[]={0.85f,2.4f};
static const float32_t coefficients_4mm[]={0.87f,1.87f};
static const float32_t coefficients_5_65mm[]={0.6748f,1.754f};
static const float32_t coefficients_7mm[]={0.6f,1.3f};
static const float32_t coefficients_15mm[]={0.9f,1.05f};
static const float32_t coefficients_21_2mm[]={0.6f,1.2f};
static void BeamFormer(pcm_sample *ptrBufferIn1, pcm_sample *ptrBufferIn2, pcm_sample *ptrBufferOut, uint32_t nNumSamples, TPCMBF *Param, libBeamforming_Handler_Internal * BeamFormingInternal);
static void BeamFormer_SetGain(libBeamforming_Handler_Internal * BeamFormingInternal);
#endif

#ifdef IIR16bitFAST
static const q15_t coefficients_4mm[]={26957 ,0,      31209   ,    12038   ,   -20212    ,   -3965     ,  16384    ,  -14207      ,   418    ,   26673    ,  -10915};
static const q15_t coefficients_15mm[]={14681   ,0,     2204   ,   -11036   ,     5752   ,     2581   ,    16384     ,    843     ,   2147     ,   884   ,    13467};
static const q15_t coefficients_5_65mm[]={18721,0,-17560,8392,19651,-8638,17820,0,-27895,13255,28921,-13665};    
static const q15_t coefficients_21_2mm[]={14681   ,0,     2204   ,   -11036   ,     5752   ,     2581   ,    16384     ,    843     ,   2147     ,   884   ,    13467};  
static void BeamFormer_1(pcm_sample *ptrBufferIn1, pcm_sample *ptrBufferIn2, pcm_sample *ptrBufferOut, uint32_t nNumSamples, TPCMBF *Param, libBeamforming_Handler_Internal * BeamFormingInternal);
static void BeamFormer_2(pcm_sample *ptrBufferIn1, pcm_sample *ptrBufferIn2, pcm_sample *ptrBufferOut, uint32_t nNumSamples, TPCMBF *Param, libBeamforming_Handler_Internal * BeamFormingInternal);
#endif

/*FIRST STEP*/
/*PDM*/
static uint8_t FirstStepInternal1(void *pM1, void *pM2, int16_t *ptr_Out, libBeamforming_Handler_Internal * BeamFormingInternal);
/*PCM*/
static uint8_t FirstStepInternal2(void *pM1, void *pM2, int16_t *ptr_Out, libBeamforming_Handler_Internal * BeamFormingInternal);
/*PDMtoPCM*/
static uint8_t FirstStepInternal3(void *pM1, void *pM2, int16_t *ptr_Out, libBeamforming_Handler_Internal * BeamFormingInternal);
static uint8_t FirstStepInternal_NoDelay(void *pM1, void *pM2, int16_t *ptr_Out, libBeamforming_Handler_Internal * BeamFormingInternal);


#endif  /*__DEFINES_H*/

