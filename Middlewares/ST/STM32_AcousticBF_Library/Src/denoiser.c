/**
******************************************************************************
* @file    denoiser.c
* @author  SRA
* @brief   Denoiser
******************************************************************************
* @attention
*
* Copyright (C) 2003 Epic Games (written by Jean-Marc Valin)
* Copyright (C) 2004-2006 Epic Games
*
* File: preprocess.c
* Preprocessor with denoising based on the algorithm by Ephraim and Malah
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
* Recommended papers:
* 
* Y. Ephraim and D. Malah, "Speech enhancement using minimum mean-square error
* short-time spectral amplitude estimator". IEEE Transactions on Acoustics,
* Speech and Signal Processing, vol. ASSP-32, no. 6, pp. 1109-1121, 1984.
* 
* Y. Ephraim and D. Malah, "Speech enhancement using minimum mean-square error
* log-spectral amplitude estimator". IEEE Transactions on Acoustics, Speech and
* Signal Processing, vol. ASSP-33, no. 2, pp. 443-445, 1985.
* 
* I. Cohen and B. Berdugo, "Speech enhancement for non-stationary noise environments".
* Signal Processing, vol. 81, no. 2, pp. 2403-2418, 2001.
* Stefan Gustafsson, Rainer Martin, Peter Jax, and Peter Vary. "A psychoacoustic
* approach to combined acoustic echo cancellation and noise reduction". IEEE
* Transactions on Speech and Audio Processing, 2002.
* 
* J.-M. Valin, J. Rouat, and F. Michaud, "Microphone array post-filter for separation
* of simultaneous non-stationary sources". In Proceedings IEEE International
* Conference on Acoustics, Speech, and Signal Processing, 2004.
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
#ifndef __DENOISER_C
#define __DENOISER_C



#ifndef M_PI
#define M_PI 3.14159263
#endif

#define LOUDNESS_EXP 5.f
#define AMP_SCALE .001f
#define AMP_SCALE_1 1000.f

#define SPEECH_PROB_START_DEFAULT       QCONST16(0.35f,15)
#define SPEECH_PROB_CONTINUE_DEFAULT    QCONST16(0.20f,15)
#define NOISE_SUPPRESS_DEFAULT       -15
#define ECHO_SUPPRESS_DEFAULT        -40
#define ECHO_SUPPRESS_ACTIVE_DEFAULT -15

#ifndef NULL
#define NULL 0
#endif

#define SQR(x) ((x)*(x))
#define SQR16(x) (MULT16_16((x),(x)))
#define SQR16_Q15(x) (MULT16_16_Q15((x),(x)))

#define DIV32_16_Q8(a,b) ((a)/(b))
#define DIV32_16_Q15(a,b) ((a)/(b))
#define SNR_SCALING 1.f
#define SNR_SCALING_1 1.f
#define SNR_SHIFT 0
#define FRAC_SCALING 1.f
#define FRAC_SCALING_1 1.f
#define FRAC_SHIFT 0
#define NOISE_SHIFT 0
#define EXPIN_SCALING 1.f
#define EXPIN_SCALING_1 1.f
#define EXPOUT_SCALING_1 1.f

static void conj_window(spx_word16_t *w, int32_t len)
{
  int32_t i;
  for (i=0;i<len;i++)
  {
    spx_word16_t tmp;
#ifdef FIXED_POINT
    spx_word16_t x = DIV32_16(MULT16_16(32767,i),len);
#else      
    spx_word16_t x = DIV32_16(MULT16_16(QCONST16(4.f,13),i),len);
#endif
    int32_t inv=0;
    if (x<QCONST16(1.f,13))
    {
    }
    else if (x<QCONST16(2.f,13))
    {
      x=QCONST16(2.f,13)-x;
      inv=1;
    }
    else if (x<QCONST16(3.f,13))
    {
      x=x-QCONST16(2.f,13);
      inv=1;
    }
    else
    {
      x=QCONST16(2.f,13)-x+QCONST16(2.f,13); /* 4 - x */
    }
    x = MULT16_16_Q14(QCONST16(1.271903f,14), x);
    tmp = SQR16_Q15(QCONST16(0.5f,15.0f)-MULT16_16_P15(QCONST16(0.5f,15.0f),spx_cos_norm(SHL32(EXTEND32(x),2))));
    if (inv == 1)
    {
      tmp=SUB16(Q15_ONE,tmp);
    }
    w[i]=spx_sqrt(SHL32(EXTEND32(tmp),15));
  }
}

/* This function approximates the gain function 
y = gamma(1.25)^2 * M(-.25;1;-x) / sqrt(x)  
which multiplied by xi/(1+xi) is the optimal gain
in the loudness domain ( sqrt[amplitude] )
*/
static inline spx_word32_t hypergeom_gain(spx_word32_t xx)
{
  int32_t ind;
  float32_t integer, frac;
  float32_t x;
  static const float32_t table[21] =
  {
    0.82157f, 1.02017f, 1.20461f, 1.37534f, 1.53363f, 1.68092f, 1.81865f,
    1.94811f, 2.07038f, 2.18638f, 2.29688f, 2.40255f, 2.50391f, 2.60144f,
    2.69551f, 2.78647f, 2.87458f, 2.96015f, 3.04333f, 3.12431f, 3.20326f
  };
  x = xx;
  integer = (float32_t)floor((float32_t)(2.0f*x));
  ind = (int32_t)integer;
  if (ind<0)
  {
    return 1.0f;
  }
  if (ind>19)
  {
    return (1.0f+(0.1296f/x));
  }
  frac = (2.0f*x)-integer;
  return ( ( (1.0f-frac) * table[ind]) + (frac*table[ind+1]) )/spx_sqrt(x+0.0001f);
}

static inline spx_word16_t qcurve(spx_word16_t x)
{
  return 1.f/(1.f+(.15f/(SNR_SCALING_1*x)));
}

static void compute_gain_floor(float32_t noise_suppress, float32_t effective_echo_suppress, spx_word32_t *noise, spx_word32_t *echo, spx_word16_t *gain_floor, int32_t len)
{
  int32_t i;
  float32_t echo_floor;
  float32_t noise_floor;
  
  noise_floor = expf(.2302585f*noise_suppress);
  echo_floor = expf(.2302585f*effective_echo_suppress);
  
  /* Compute the gain floor based on different floors for the background noise and residual echo */
  for (i=0;i<len;i++)
  {
    gain_floor[i] = spx_sqrt( ( (noise_floor*noise[i]) + (echo_floor*echo[i]) )/(1.0f+noise[i] + echo[i]) );
  }
}

static void denoiserstate_init(SpeexPreprocessState * st,int32_t frame_size, int32_t sampling_rate)
{
  int32_t i;
  int32_t N, N3, N4, M;
  
  /**SpeexPreprocessState *st = (SpeexPreprocessState *)speex_alloc(sizeof(SpeexPreprocessState));**/
  st->frame_size = frame_size;
  
  /* Round ps_size down to the nearest power of two */
  
#if 0
  i=1;
  st->ps_size = st->frame_size;
  while(1)
  {
    if (st->ps_size & ~i)
    {
      st->ps_size &= ~i;
      i<<=1;
    }
    else
    {
      break;
    }
  }
  if (st->ps_size < 3*st->frame_size/4)
  {
    st->ps_size = st->ps_size * 3 / 2;
  }
#else
  st->ps_size = st->frame_size;
#endif
  
  N = st->ps_size;
  N3 = (2*N) - st->frame_size;
  N4 = st->frame_size - N3;
  
  st->sampling_rate = sampling_rate;
  st->denoise_enabled = 1;
  st->vad_enabled = 0;
  st->dereverb_enabled = 0;
  st->reverb_decay = 0.0f;
  st->reverb_level = 0.0f;
  st->noise_suppress = NOISE_SUPPRESS_DEFAULT;
  st->echo_suppress = ECHO_SUPPRESS_DEFAULT;
  st->echo_suppress_active = ECHO_SUPPRESS_ACTIVE_DEFAULT;
  st->speech_prob_start = SPEECH_PROB_START_DEFAULT;
  st->speech_prob_continue = SPEECH_PROB_CONTINUE_DEFAULT;
  st->echo_state = NULL;
  st->nbands = NB_BANDS;
  
  M = st->nbands;
  
  conj_window(st->window, 2*N3);
  for (i=2*N3; i<(2*st->ps_size); i++)
  {
    if (i<(2*NN_MAX))
    {
      st->window[i]=Q15_ONE;
    }
  }
  
  if (N4>0)
  {
    for (i=N3-1;i>=0;i--)
    {
      st->window[i+N3+N4]=st->window[i+N3];
      st->window[i+N3]=1.0f;
    }
  }
  /**st->window = window_denoiser;**/
  for (i=0; i<(N+M); i++)
  {
    st->noise[i]=QCONST32(1.f,NOISE_SHIFT);
    /**st->reverb_estimate[i]=0;**/
    st->old_ps[i]=1.0f;
    st->gain[i]=Q15_ONE;
    st->post[i]=SHL16(1.0f, SNR_SHIFT);
    st->prior[i]=SHL16(1.0f, SNR_SHIFT);
  }
  
  for (i=0;i<N;i++)
  {
    st->update_prob[i] = 1;
  }
  
  for (i=0;i<N3;i++)
  {
    st->inbuf[i]=0.0f;
    st->outbuf[i]=0.0f;
  }
  
  st->agc_enabled = 0;
  st->agc_level = 8000.0f;
  /**st->loudness = pow(AMP_SCALE*st->agc_level,LOUDNESS_EXP);**/
  st->loudness = 1e-15f;
  st->agc_gain = 1.0f;
  st->max_gain = 30.0f;
  st->max_increase_step = expf(0.11513f * 12.f*(float32_t)st->frame_size / (float32_t)st->sampling_rate);
  st->max_decrease_step = expf(-0.11513f * 40.f*(float32_t)st->frame_size / (float32_t)st->sampling_rate);
  st->prev_loudness = 1.0f;
  st->init_max = 1.0f;
  st->was_speech = 0;
  st->nb_adapt=0;
  st->min_count=0;
}

static void denoiser_analize(SpeexPreprocessState *st, spx_int16_t *x)
{
  int32_t i;
  int32_t N = st->ps_size;
  int32_t N3 = (2*N) - st->frame_size;
  int32_t N4 = st->frame_size - N3;
  spx_word32_t *ps=st->ps;
  
  /* 'Build' input frame */
  for (i=0;i<N3;i++)
  {
    st->frame[i]=st->inbuf[i];
  }
  
  for (i=0;i<st->frame_size;i++)
  {
    st->frame[N3+i]=(float32_t)x[i];
  }
  
  /* Update inbuf */
  for (i=0;i<N3;i++)
  {
    st->inbuf[i]=(float32_t)x[N4+i];
  }
  
  /* Windowing */
  arm_mult_f32((float32_t *)st->window,st->frame,st->frame,((uint32_t)N/2U));
  
  /* Perform FFT */
  fft((st->fft_lookup), st->frame, st->ft);
  
  /* Power spectrum */
  ps[0]=MULT16_16(st->ft[0],st->ft[0]);
  arm_cmplx_mag_squared_f32(&st->ft[1],&ps[1],((uint32_t)N/2U)-1U);
  
  filterbank_compute_bank32(st->bank, ps, ps+N);
}

static void update_noise_prob(SpeexPreprocessState *st)
{
  int32_t i;
  int32_t min_range;
  int32_t N = st->ps_size;
  
  for (i=1; i<(N-1); i++)
  {
    st->S[i] =  MULT16_32_Q15(QCONST16(.8f,15),st->S[i]) + MULT16_32_Q15(QCONST16(.05f,15),st->ps[i-1])
      + MULT16_32_Q15(QCONST16(.1f,15),st->ps[i]) + MULT16_32_Q15(QCONST16(.05f,15),st->ps[i+1]);
  }
  st->S[0] =  MULT16_32_Q15(QCONST16(.8f,15),st->S[0]) + MULT16_32_Q15(QCONST16(.2f,15),st->ps[0]);
  st->S[N-1] =  MULT16_32_Q15(QCONST16(.8f,15),st->S[N-1]) + MULT16_32_Q15(QCONST16(.2f,15),st->ps[N-1]);
  
  if (st->nb_adapt==1)
  {
    for (i=0;i<N;i++)
    {
      st->Smin[i] = 0.0f;
      st->Stmp[i] = 0.0f;
    }
  }
  
  if (st->nb_adapt < 100)
  {
    min_range = 15;
  }
  else if (st->nb_adapt < 1000)
  {
    min_range = 50;
  }
  else if (st->nb_adapt < 10000)
  {
    min_range = 150;
  }
  else
  {
    min_range = 300;
  }
  if (st->min_count > min_range)
  {
    st->min_count = 0;
    for (i=0;i<N;i++)
    {
      st->Smin[i] = MIN32(st->Stmp[i], st->S[i]);
      st->Stmp[i] = st->S[i];
    }
  }
  else
  {
    for (i=0;i<N;i++)
    {
      st->Smin[i] = MIN32(st->Smin[i], st->S[i]);
      st->Stmp[i] = MIN32(st->Stmp[i], st->S[i]);
    }
  }
  for (i=0;i<N;i++)
  {
    if (MULT16_32_Q15(QCONST16(.4f,15),st->S[i]) > st->Smin[i])
    {
      st->update_prob[i] = 1;
    }
    else
    {
      st->update_prob[i] = 0;
    }
  }
}

#define NOISE_OVERCOMPENS 1.

void adaptiveget_residual(SpeexEchoState *st, spx_word32_t *Yout, int32_t len);
static int32_t denoiser_A_run(SpeexPreprocessState *st, spx_int16_t *x)
{
  int32_t i;
  int32_t M;
  int32_t N = st->ps_size;
  int32_t N3 = (2*N) - st->frame_size;
  int32_t N4 = st->frame_size - N3;
  spx_word32_t *ps=st->ps;
  spx_word32_t Zframe;
  spx_word16_t Pframe;
  spx_word16_t beta, beta_1;
  spx_word16_t effective_echo_suppress;
  
  st->nb_adapt++;
  if (st->nb_adapt>20000)
  {
    st->nb_adapt = 20000;
  }
  st->min_count++;
  
  beta = MAX16(QCONST16(.03f,15),DIV32_16(Q15_ONE,st->nb_adapt));
  beta_1 = Q15_ONE-beta;
  M = st->nbands;
  /* Deal with residual echo if provided */
  if (st->echo_state != NULL)
  {
    adaptiveget_residual(st->echo_state, st->residual_echo, N);
    
    /* If there are NaNs or ridiculous values, it'll show up in the DC and we just reset everything to zero */
    if ( !( (st->residual_echo[0] >=0.0f) && (st->residual_echo[0]<((float32_t)N*1e9f)) ) )
    {
      for (i=0;i<N;i++)
      {
        st->residual_echo[i] = 0.0f;
      }
    }
    
    for (i=0;i<N;i++)
    {
      st->echo_noise[i] = MAX32(MULT16_32_Q15(QCONST16(.6f,15),st->echo_noise[i]), st->residual_echo[i]);
    }
    
    filterbank_compute_bank32(st->bank, st->echo_noise, st->echo_noise+N); //OPTIMIZE
  }
  else
  {
    for (i=0; i<(N+M); i++)
    {
      st->echo_noise[i] = 0.0f;
    }
  }
  
  denoiser_analize((SpeexPreprocessState *)st, x); //SOME OPTIMIZATION USING ARM, STILL TO BE OPTIMIZED (fft)
  update_noise_prob((SpeexPreprocessState *)st); //OPTIMIZE
  
  /* Update the noise estimate for the frequencies where it can be */
  for (i=0;i<N;i++)
  {
    if ( (st->update_prob[i] == 0) || (st->ps[i] < PSHR32(st->noise[i], NOISE_SHIFT)) )
    {
      st->noise[i] = MAX32(EXTEND32(0),MULT16_32_Q15(beta_1,st->noise[i]) + MULT16_32_Q15(beta,SHL32(st->ps[i],NOISE_SHIFT)));
    }
  }
  
  filterbank_compute_bank32(st->bank, st->noise, st->noise+N); //OPTIMIZE
  
  
  /* Special case for first frame */
  if (st->nb_adapt==1)
  {
    for (i=0;i<(N+M);i++)
    {
      st->old_ps[i] = ps[i];
    }
  }
  
  /* Compute a posteriori SNR */
  for (i=0;i<(N+M);i++)
  {
    spx_word16_t gamma;
    /* Total noise estimate including residual echo and reverberation */
    spx_word32_t tot_noise = 1.0f + st->noise[i]+ st->echo_noise[i];
    
    /* A posteriori SNR = ps/noise - 1*/
    st->post[i] = (ps[i]/tot_noise)-1.0f;
    st->post[i]=MIN16(st->post[i], 100.0f );
    
    /* Computing update gamma = .1 + .9*(old/(old+noise))^2 */
    gamma =0.1f+(0.89f * SQR16_Q15((st->old_ps[i]/(st->old_ps[i]+tot_noise))));
    
    /* A priori SNR update = gamma*max(0,post) + (1-gamma)*old/noise */
    st->prior[i] = (gamma * (MAX16(0.0f,st->post[i]))) + ((1.0f-gamma)*(st->old_ps[i]/tot_noise));
    st->prior[i] = MIN16(st->prior[i], 100.0f);
  }
  
  /* Recursive average of the a priori SNR. A bit smoothed for the psd components */
  st->zeta[0] = (0.7f*st->zeta[0])+ (0.3f*st->prior[0]);
  
  for (i=1;i<(N-1);i++)
  {
    st->zeta[i] = (0.7f*st->zeta[i]) + (0.15f*st->prior[i])+(0.75f * st->prior[i-1]) + (0.75f * st->prior[i+1]);
  }
  
  for (i=N-1;i<(N+M);i++)
  {
    st->zeta[i] = (0.7f*st->zeta[i]) + (0.3f*st->prior[i]);
  }
  
  /* Speech probability of presence for the entire frame is based on the average filterbank a priori SNR */
  Zframe = 0.0f;
  for (i=N;i<(N+M);i++)
  {
    Zframe = (Zframe + st->zeta[i]);
  }
  Pframe = 0.1f +(0.899f*qcurve(Zframe/(float32_t)st->nbands));
  effective_echo_suppress = ((1.0f-Pframe)* (float32_t)st->echo_suppress)+ (Pframe* (float32_t)st->echo_suppress_active);
  compute_gain_floor((float32_t)st->noise_suppress, effective_echo_suppress, st->noise+N, st->echo_noise+N, st->gain_floor+N, M); //OPTIMIZED REMOVING M (24) SQRTs
  
  /* Compute Ephraim & Malah gain speech probability of presence for each critical band (Bark scale)
  Technically this is actually wrong because the EM gaim assumes a slightly different probability 
  distribution */
  for (i=N;i<(N+M);i++)
  {
    /* See EM and Cohen papers*/
    spx_word32_t theta;
    /* Gain from hypergeometric function */
    spx_word32_t MM;
    /* Weiner filter gain */
    spx_word16_t prior_ratio;
    /* a priority probability of speech presence based on Bark sub-band alone */
    spx_word16_t P1;
    /* Speech absence a priori probability (considering sub-band and frame) */
    spx_word16_t q;
    
    prior_ratio = PDIV32_16(SHL32(EXTEND32(st->prior[i]), 15), ADD16(st->prior[i], SHL32(1,SNR_SHIFT)));
    theta = MULT16_32_P15(prior_ratio, QCONST32(1.f,EXPIN_SHIFT)+SHL32(EXTEND32(st->post[i]),EXPIN_SHIFT-SNR_SHIFT));
    
    MM = hypergeom_gain(theta);  //REMOVED LOOKUP TABLE FROM FLASH
    /* Gain with bound */
    st->gain[i] = EXTRACT16(MIN32(Q15_ONE, MULT16_32_Q15(prior_ratio, MM)));
    /* Save old Bark power spectrum */
    st->old_ps[i] = MULT16_32_P15(QCONST16(.2f,15),st->old_ps[i]) + MULT16_32_P15(MULT16_16_P15(QCONST16(.8f,15),SQR16_Q15(st->gain[i])),ps[i]);
    
    P1 = QCONST16(.199f,15)+MULT16_16_Q15(QCONST16(.8f,15),qcurve (st->zeta[i]));
    q = Q15_ONE-MULT16_16_Q15(Pframe,P1);
    
    st->gain2[i]=1.0f/( 1.f + ((q/(1.f-q))*(1.0f+st->prior[i])*expf(-theta)) );
  }
  
  /* Convert the EM gains and speech prob to linear frequency */ //440us
  filterbank_compute_psd16(st->bank,st->gain2+N, st->gain2);
  filterbank_compute_psd16(st->bank,st->gain+N, st->gain);
  
  /* Use linear gain resolution */
  for (i=N;i<(N+M);i++)
  {
    spx_word16_t tmp;
    spx_word16_t p = st->gain2[i];
    st->gain[i] = MAX16(st->gain[i], st->gain_floor[i]);// CHECKING THIS RESULT WE CAN PERFORM 1 SQRT LESS IN THE NEXT INSTRUCTION
    tmp = (p*spx_sqrt(st->gain[i])) + ((1.0f - p)*spx_sqrt(st->gain_floor[i]) );
    st->gain2[i]=SQR16_Q15(tmp);
  }
  
  filterbank_compute_psd16(st->bank,st->gain2+N, st->gain2);
  
  /* Apply computed gain */ //180us
  for (i=1;i<N;i++)
  {
    st->ft[(2*i)-1] = MULT16_16_P15(st->gain2[i],st->ft[(2*i)-1]);
    st->ft[2*i] = MULT16_16_P15(st->gain2[i],st->ft[2*i]);
  }
  st->ft[0] = MULT16_16_P15(st->gain2[0],st->ft[0]);
  st->ft[(2*N)-1] = MULT16_16_P15(st->gain2[N-1],st->ft[(2*N)-1]);
  
  /* Inverse FFT with 1/N scaling */
  ifft(st->fft_lookup, st->ft, st->frame);
  /* Synthesis window (for WOLA) */
  arm_mult_f32((float32_t *)st->window,st->frame,st->frame,(uint32_t)N*2U);
  
  /* Perform overlap and add */
  for (i=0;i<N3;i++)
  {
    x[i] = (spx_int16_t)st->outbuf[i] + (spx_int16_t)st->frame[i];
  }
  for (i=0;i<N4;i++)
  {
    x[N3+i] = (spx_int16_t)(st->frame[N3+i]);
  }
  
  /* Update outbuf */
  for (i=0;i<N3;i++)
  {
    st->outbuf[i] = st->frame[st->frame_size+i];
  }
  
  /* FIXME: This VAD is a kludge */
  st->speech_prob = Pframe;
  
  return 1;
}

static int32_t denoiser_setup(SpeexPreprocessState *state, int32_t request, SpeexEchoState *ptr)
{
  int32_t ret = 0;
  SpeexPreprocessState *st;
  st=(SpeexPreprocessState*)state;
  switch(request)
  {
  case SPEEX_PREPROCESS_SET_ECHO_STATE:
    st->echo_state = ptr;
    break;
  default:
    ret = -1;
    break;
  }
  return ret;
}


#endif  /*__DENOISER_C*/

