/**
******************************************************************************
* @file    defines_ec.h
* @author  SRA
* @version v3.0.0
* @date    8-Oct-2021
* @brief   This file contains Acoustic Echo Cancellation library definitions.
******************************************************************************
* @attention
*
* Copyright (C) 2003 Epic Games (written by Jean-Marc Valin)
* Copyright (C) 2004-2006 Epic Games
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
#ifndef __DEFINES_EC_H
#define __DEFINES_EC_H

/* Includes ------------------------------------------------------------------*/
#include "math.h"

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
#define N_SPEAKER_MAX 1
#define SPX_EC_FS_16000

#ifdef SPX_EC_FS_8000
  #define ECHO_BUFF 128U
  #define NN_MAX 128
  #define NN 128
  #define FS (uint16_t)8000
#elif defined (SPX_EC_FS_16000)
  #define ECHO_BUFF 128U
  #define NN_MAX 128
  #define NN 128
  #define FS (uint16_t)16000
#elif defined (SPX_EC_FS_32000)
  #define ECHO_BUFF (32*8)
  #define NN_MAX (32*8)
  #define NN (32*8)
  #define FS (uint16_t)32000
#elif defined (SPX_EC_FS_48000)
  #define ECHO_BUFF (48*8)
  #define NN_MAX (48*8)
  #define NN (48*8)
  #define FS (uint16_t)48000
#else
  #error "SPX_EC_FS_8000 or SPX_EC_FS_16000 or SPX_EC_FS_32000 or SPX_EC_FS_48000 must be defined"
#endif



#define NB_BANDS 24

#define SPEEX_ECHO_GET_FRAME_SIZE 3

/** Set sampling rate */
#define SPEEX_ECHO_SET_SAMPLING_RATE 24
/** Get sampling rate */
#define SPEEX_ECHO_GET_SAMPLING_RATE 25

/* Can't set window sizes */
/** Get size of impulse response (int32) */
#define SPEEX_ECHO_GET_IMPULSE_RESPONSE_SIZE 27

/* Can't set window content */
/** Get impulse response (int32[]) */
#define SPEEX_ECHO_GET_IMPULSE_RESPONSE 29

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
#define ABS32(x) ( (((float32_t)(x)) < 0.0f) ? (-((float32_t)(x))) : ((float32_t)(x)))    /**< Absolute 32-bit value.  */
#define MIN32(a,b) ( (((float32_t)(a)) < ((float32_t)(b))) ? ((float32_t)(a)) : ((float32_t)(b)))   /**< Maximum 32-bit floating-point value.   */
#define MAX32(a,b) ( (((float32_t)(a)) > ((float32_t)(b))) ? ((float32_t)(a)) : ((float32_t)(b)))   /**< Maximum 32-bit floating-point value.   */
#define MIN32FIXED(a,b) ( ((a) < (b)) ? (a) : (b))   /**< Maximum 32-bit fixed-point value.   */
#define MAX32FIXED(a,b) ( ((a) > (b)) ? (a) : (b))   /**< Maximum 32-bit fixed-point value.   */

#ifndef UNUSED
  #define UNUSED(X) (void)(X)      /* To avoid gcc/g++ warnings */
#endif

#define QCONST16(x,bits) (x)
#define QCONST32(x,bits) (x)

#define NEG16(x) (-(x))
#define NEG32(x) (-((float32_t)(x)))
#define EXTRACT16(x) (x)
#define EXTEND32(x) ((float32_t)(x))
#define SHR16(a,shift) (a)
#define SHL16(a,shift) (a)
#define SHR32(a,shift) ((float32_t)(a))
#define SHL32(a,shift) ((float32_t)(a))
#define PSHR16(a,shift) (a)
#define PSHR32(a,shift) ((float32_t)(a))
#define VSHR32(a,shift) ((float32_t)(a))
#define SATURATE16(x,a) (x)
#define SATURATE32(x,a) ((float32_t)(x))

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

/** Set preprocessor denoiser state */
#define SPEEX_PREPROCESS_SET_DENOISE 0
/** Get preprocessor denoiser state */
#define SPEEX_PREPROCESS_GET_DENOISE 1

/** Set preprocessor Automatic Gain Control state */
#define SPEEX_PREPROCESS_SET_AGC 2
/** Get preprocessor Automatic Gain Control state */
#define SPEEX_PREPROCESS_GET_AGC 3

/** Set preprocessor Voice Activity Detection state */
#define SPEEX_PREPROCESS_SET_VAD 4
/** Get preprocessor Voice Activity Detection state */
#define SPEEX_PREPROCESS_GET_VAD 5

/** Set preprocessor Automatic Gain Control level (float) */
#define SPEEX_PREPROCESS_SET_AGC_LEVEL 6
/** Get preprocessor Automatic Gain Control level (float) */
#define SPEEX_PREPROCESS_GET_AGC_LEVEL 7

/** Set preprocessor dereverb state */
#define SPEEX_PREPROCESS_SET_DEREVERB 8
/** Get preprocessor dereverb state */
#define SPEEX_PREPROCESS_GET_DEREVERB 9

/** Set preprocessor dereverb level */
#define SPEEX_PREPROCESS_SET_DEREVERB_LEVEL 10
/** Get preprocessor dereverb level */
#define SPEEX_PREPROCESS_GET_DEREVERB_LEVEL 11

/** Set preprocessor dereverb decay */
#define SPEEX_PREPROCESS_SET_DEREVERB_DECAY 12
/** Get preprocessor dereverb decay */
#define SPEEX_PREPROCESS_GET_DEREVERB_DECAY 13

/** Set probability required for the VAD to go from silence to voice */
#define SPEEX_PREPROCESS_SET_PROB_START 14
/** Get probability required for the VAD to go from silence to voice */
#define SPEEX_PREPROCESS_GET_PROB_START 15

/** Set probability required for the VAD to stay in the voice state (integer percent) */
#define SPEEX_PREPROCESS_SET_PROB_CONTINUE 16
/** Get probability required for the VAD to stay in the voice state (integer percent) */
#define SPEEX_PREPROCESS_GET_PROB_CONTINUE 17

/** Set maximum attenuation of the noise in dB (negative number) */
#define SPEEX_PREPROCESS_SET_NOISE_SUPPRESS 18
/** Get maximum attenuation of the noise in dB (negative number) */
#define SPEEX_PREPROCESS_GET_NOISE_SUPPRESS 19

/** Set maximum attenuation of the residual echo in dB (negative number) */
#define SPEEX_PREPROCESS_SET_ECHO_SUPPRESS 20
/** Get maximum attenuation of the residual echo in dB (negative number) */
#define SPEEX_PREPROCESS_GET_ECHO_SUPPRESS 21

/** Set maximum attenuation of the residual echo in dB when near end is active (negative number) */
#define SPEEX_PREPROCESS_SET_ECHO_SUPPRESS_ACTIVE 22
/** Get maximum attenuation of the residual echo in dB when near end is active (negative number) */
#define SPEEX_PREPROCESS_GET_ECHO_SUPPRESS_ACTIVE 23

/** Set the corresponding echo canceller state so that residual echo suppression can be performed (NULL for no residual echo suppression) */
#define SPEEX_PREPROCESS_SET_ECHO_STATE 24
/** Get the corresponding echo canceller state */
#define SPEEX_PREPROCESS_GET_ECHO_STATE 25

/** Set maximal gain increase in dB/second (int32) */
#define SPEEX_PREPROCESS_SET_AGC_INCREMENT 26

/** Get maximal gain increase in dB/second (int32) */
#define SPEEX_PREPROCESS_GET_AGC_INCREMENT 27

/** Set maximal gain decrease in dB/second (int32) */
#define SPEEX_PREPROCESS_SET_AGC_DECREMENT 28

/** Get maximal gain decrease in dB/second (int32) */
#define SPEEX_PREPROCESS_GET_AGC_DECREMENT 29

/** Set maximal gain in dB (int32) */
#define SPEEX_PREPROCESS_SET_AGC_MAX_GAIN 30

/** Get maximal gain in dB (int32) */
#define SPEEX_PREPROCESS_GET_AGC_MAX_GAIN 31

/*  Can't set loudness */
/** Get loudness */
#define SPEEX_PREPROCESS_GET_AGC_LOUDNESS 33

/*  Can't set gain */
/** Get current gain (int32 percent) */
#define SPEEX_PREPROCESS_GET_AGC_GAIN 35

/*  Can't set spectrum size */
/** Get spectrum size for power spectrum (int32) */
#define SPEEX_PREPROCESS_GET_PSD_SIZE 37

/*  Can't set power spectrum */
/** Get power spectrum (int32[] of squared values) */
#define SPEEX_PREPROCESS_GET_PSD 39

/*  Can't set noise size */
/** Get spectrum size for noise estimate (int32)  */
#define SPEEX_PREPROCESS_GET_NOISE_PSD_SIZE 41

/*  Can't set noise estimate */
/** Get noise estimate (int32[] of squared values) */
#define SPEEX_PREPROCESS_GET_NOISE_PSD 43

/* Can't set speech probability */
/** Get speech probability in last frame (int32).  */
#define SPEEX_PREPROCESS_GET_PROB 45

/** Set preprocessor Automatic Gain Control level (int32) */
#define SPEEX_PREPROCESS_SET_AGC_TARGET 46
/** Get preprocessor Automatic Gain Control level (int32) */
#define SPEEX_PREPROCESS_GET_AGC_TARGET 47

#define spx_sqrt my_sqrt  //sqrt
#define spx_acos acosf
#define spx_exp expf
#define spx_cos_norm(x) (cosf((.5f*M_PI)*(x)))
#define spx_atan atanf

static inline float32_t my_sqrt(float32_t a)
{
  float32_t b;
  (void)arm_sqrt_f32(a, &b);
  return b;
}

/** Discrete Rotational Fourier Transform lookup */
typedef struct
{
  int32_t n;
  float32_t trigcache[2 * ECHO_BUFF * 3];
  int32_t splitcache[32];
} drft_lookup;

typedef struct
{
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

typedef struct
{
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

  spx_word16_t e[N_MIC_MAX * NN_MAX * 2];
  spx_word16_t x[N_SPEAKER_MAX * NN_MAX * 2];   /* Far-end input buffer (2N): x */
  #ifdef USE_ARM_FFT
  spx_word16_t X[N_SPEAKER_MAX * NN_MAX * 2 * 2];
  #else
  spx_word16_t *X;      /* Far-end buffer (M+1 frames) in frequency domain */

  #endif
  spx_word16_t input[N_MIC_MAX * NN_MAX];       /* scratch: input */
  spx_word16_t y[N_MIC_MAX * NN_MAX * 2];       /* scratch: y */
  spx_word16_t last_y[N_MIC_MAX * NN_MAX * 2];  /* last_y */
  #ifdef USE_ARM_FFT
  spx_word16_t Y[N_MIC_MAX * NN_MAX * 2 * 2]; //*Y;      /* scratch */
  spx_word16_t E[N_MIC_MAX * NN_MAX * 2 * 2]; //*E;
  #else
  spx_word16_t Y[N_MIC_MAX * NN_MAX * 2];       /* scratch: Y */
  spx_word16_t E[N_MIC_MAX * NN_MAX * 2];       /* E */

  #endif
  spx_word32_t PHI[NN_MAX * 2];                 /* scratch: PHI */
  spx_word32_t *W;
  spx_word16_t *foreground; /* Foreground filter weights */

  spx_word32_t  Davg1;  /* 1st recursive average of the residual power difference */
  spx_word32_t  Davg2;  /* 2nd recursive average of the residual power difference */
  spx_float_t   Dvar1;  /* Estimated variance of 1st estimator */
  spx_float_t   Dvar2;  /* Estimated variance of 2nd estimator */
  spx_word32_t power[NN_MAX + 1];               /* Power of the far-end signal: power */
  spx_float_t  power_1[NN_MAX + 1];             /* Inverse power of far-end: power_1 */
  spx_word16_t wtmp[NN_MAX];                    /*wtmp */
  #ifdef FIXED_POINT
  spx_word16_t wtmp2[NN_MAX + 1]; //*wtmp2;  /* scratch */
  #endif
  spx_word32_t Rf[(NN_MAX + 1)];                /*Rf */
  spx_word32_t Yf[(NN_MAX + 1)];                /*Yf */
  spx_word32_t Xf[(NN_MAX + 1)];                /*Xf */
  spx_word32_t Eh[(NN_MAX + 1)];                /*Eh */
  spx_word32_t Yh[(NN_MAX + 1)];                /*Yh */
  spx_float_t   Pey;
  spx_float_t   Pyy;
  spx_word16_t window[NN_MAX * 2];              /*window */
  spx_word16_t *prop;
  drft_lookup *fft_table;
  spx_word16_t memX;
  spx_word16_t memD;
  spx_word16_t memE;
  spx_word16_t preemph;
  spx_word16_t notch_radius;
  spx_mem_t notch_mem[2 * N_MIC_MAX];           /*notch_mem */

  spx_int16_t play_buf[1];
  int32_t play_buf_pos;
  int32_t play_buf_started;
} SpeexEchoState;

/** Speex pre-processor state. */
typedef struct
{
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

  spx_word16_t  speech_prob;  /**< Probability last frame was speech */

  /* DSP-related arrays */
  spx_word16_t frame[2 * NN_MAX];               /**< Processing frame (2*ps_size) */
  spx_word16_t ft[2 * NN_MAX];                  /**< Processing frame in freq domain (2*ps_size) */
  spx_word32_t ps[NN_MAX + NB_BANDS];           /**< Current power spectrum */
  spx_word16_t gain2[NN_MAX + NB_BANDS];        /**< Adjusted gains */
  spx_word16_t gain_floor[NN_MAX + NB_BANDS];   /**< Minimum gain allowed */
  float32_t window[2 * NN_MAX];
  spx_word32_t noise[NN_MAX + NB_BANDS];        /**< Noise estimate */
  spx_word32_t old_ps[NN_MAX + NB_BANDS];       /**< Power spectrum for last frame */
  spx_word16_t gain[NN_MAX + NB_BANDS];         /**< Ephraim Malah gain */
  spx_word16_t prior[NN_MAX + NB_BANDS];        /**< A-priori SNR */
  spx_word16_t post[NN_MAX + NB_BANDS];         /**< A-posteriori SNR */

  spx_word32_t S[NN_MAX];                       /**< Smoothed power spectrum */
  spx_word32_t Smin[NN_MAX];                    /**< See Cohen paper */
  spx_word32_t Stmp[NN_MAX];                    /**< See Cohen paper */
  int32_t update_prob[NN_MAX];                      /**< Probability of speech presence for noise update */

  spx_word16_t zeta[NN_MAX + NB_BANDS];         /**< Smoothed a priori SNR */
  spx_word32_t echo_noise[NN_MAX + NB_BANDS];
  spx_word32_t residual_echo[NN_MAX + NB_BANDS];

  /* Misc */
  spx_word16_t inbuf[NN_MAX];                   /**< Input buffer (overlapped analysis) */
  spx_word16_t outbuf[NN_MAX];                  /**< Output buffer (for overlap and add) */

  /* AGC stuff, only for floating point for now */
  #ifndef FIXED_POINT
  int32_t    agc_enabled;
  float32_t  agc_level;
  float32_t  loudness_accum;
  float32_t loudness_weight[NN_MAX];   /**< Perceptual loudness curve */
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
} SpeexPreprocessState;




/* Exported functions ------------------------------------------------------- */

//echo//
void Echo_get_residual(SpeexEchoState *st, spx_word32_t *residual_echo, int len);
int Echo_ctrl(SpeexEchoState *st, int request, void *ptr);
void Echo_cancellation(SpeexEchoState *st, const spx_int16_t *in, const spx_int16_t *far_end, spx_int16_t *out);
void Echo_init(SpeexEchoState *st, int frame_size, int filter_length, int nb_mic, int nb_speakers);

//ffts.c
void libSpeexAEC_fft_init(drft_lookup *table, int32_t size);
void libSpeexAEC_fft(drft_lookup *table, float32_t *in, float32_t *out);
void libSpeexAEC_ifft(drft_lookup *table, float32_t *in, float32_t *out);

/*FILTER BANK related*/
void filterbank_new(FilterBank *bank, int32_t banks, spx_word32_t sampling, int32_t len, int32_t type);
void filterbank_compute_psd16(FilterBank *bank, spx_word16_t *mel, spx_word16_t *ps);
void filterbank_compute_bank32(FilterBank *bank, spx_word32_t *ps, spx_word32_t *mel);

//preprocess.c
int Preprocess_setup(SpeexPreprocessState *state, int request, void *ptr);
int Preprocess(SpeexPreprocessState *st, spx_int16_t *x);
void Preprocess_init(SpeexPreprocessState *st, int frame_size, int sampling_rate);



/*SMALLFFT related*/
void drft_init(drft_lookup *l, int32_t n);
void drft_backward(drft_lookup *l, float32_t *data);
void drft_forward(drft_lookup *l, float32_t *data);

/*ARM*/
//arm_status arm_mat_mult_f32(  const arm_matrix_instance_f32 * pSrcA,  const arm_matrix_instance_f32 * pSrcB,  arm_matrix_instance_f32 * pDst);


#endif  /*__DEFINES_EC_H*/

