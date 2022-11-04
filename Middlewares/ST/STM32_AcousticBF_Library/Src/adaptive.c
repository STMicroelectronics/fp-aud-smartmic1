/**
******************************************************************************
* @file    adaptive.c
* @author  SRA
* @brief   Echo canceller based on the MDF algorithm
******************************************************************************
* @attention
*
* Copyright (C) 2003-2008 Jean-Marc Valin
*
* File: mdf.c
* Echo canceller based on the MDF algorithm (see below)
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
* The echo canceller is based on the MDF algorithm described in:
* 
* J. S. Soo, K. K. Pang Multidelay block frequency adaptive filter,
* IEEE Trans. Acoust. Speech Signal Process., Vol. ASSP-38, No. 2,
* February 1990.
* 
* We use the Alternatively Updated MDF (AUMDF) variant. Robustness to
* double-talk is achieved using a variable learning rate as described in:
* 
* Valin, J.-M., On Adjusting the Learning Rate in Frequency Domain Echo
* Cancellation With Double-Talk. IEEE Transactions on Audio,
* Speech and Language Processing, Vol. 15, No. 3, pp. 1030-1034, 2007.
* http://people.xiph.org/~jm/papers/valin_taslp2006.pdf
* 
* There is no explicit double-talk detection, but a continuous variation
* in the learning rate based on residual echo, double-talk and background
* noise.
* 
* About the fixed-point version:
* All the signals are represented with 16-bit words. The filter weights
* are represented with 32-bit words, but only the top 16 bits are used
* in most cases. The lower 16 bits are completely unreliable (due to the
* fact that the update is done only on the top bits), but help in the
* adaptation -- probably by removing a "threshold effect" due to
* quantization (rounding going to zero) when the gradient is small.
* 
* Another kludge that seems to work good: when performing the weight
* update, we only move half the way toward the "goal" this seems to
* reduce the effect of quantization noise in the update phase. This
* can be seen as applying a gradient descent on a "soft constraint"
* instead of having a hard constraint.
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
#ifndef __ADAPTIVE_C
#define __ADAPTIVE_C


#define WEIGHT_SHIFT 0
#define WORD2INT(x) ( ((x) < -32767.5f) ? -32768.0f : ( ((x) > 32766.5f) ? 32767.0f : floor((float32_t)(.5f+(x)))))

/* If enabled, the AEC will use a foreground filter and a background filter to be more robust to double-talk
and difficult signals in general. The cost is an extra FFT and a matrix-vector multiply */

#define MIN_LEAK .005f
/* Constants for the two-path filter */
#define VAR1_SMOOTH .36f
#define VAR2_SMOOTH .7225f
#define VAR1_UPDATE .5f
#define VAR2_UPDATE .25f
#define VAR_BACKTRACK 4.f
#define TOP16(x) (x)

void adaptiveget_residual(SpeexEchoState *st, spx_word32_t *Yout, int32_t len);

#define ST_FRAMESIZE 128


/* This inner product is slightly different from the codec version because of fixed-point */
static inline spx_word32_t mdf_inner_prod(const spx_word16_t *x, const spx_word16_t *y, int32_t len)
{
  spx_word32_t sum=0.0f;
  arm_dot_prod_f32((float32_t *)x, (float32_t *)y, (uint32_t)len ,&sum);
  return sum;
}

/** Compute power spectrum of a half-complex (packed) vector */
static inline void power_spectrum(const spx_word16_t *X, spx_word32_t *ps, int32_t N)
{
  ps[0]+=X[0] * X[0];
  arm_cmplx_mag_squared_f32((float32_t *)&X[1],&ps[1],((uint32_t)N/2U)-1U);
  ps[(N/2)-1]+=X[N-1] * X[N-1];
}

/** Compute power spectrum of a half-complex (packed) vector and accumulate */
static inline void power_spectrum_accum(const spx_word16_t *X, spx_word32_t *ps, int32_t N)
{
  int32_t i;
  int32_t j = 1;
  ps[0]+=MULT16_16(X[0],X[0]);
  for (i=1; i<(N-1); i+=2)
  {
    ps[j] +=  MULT16_16(X[i],X[i]) + MULT16_16(X[i+1],X[i+1]);
    j++;
  }
  
  if (i<N)
  {
    ps[j]+=MULT16_16(X[i],X[i]);
  }
}

/** Compute cross-power spectrum of a half-complex (packed) vectors and add to acc */
static inline void spectral_mul_accum(const spx_word16_t *X, const spx_word32_t *Y, spx_word16_t *acc, int32_t N, int32_t M) // N=256, M=1
{
  UNUSED(M);
  acc[0] = X[0]*Y[0];
  arm_cmplx_mult_cmplx_f32((float32_t *)&X[1], (float32_t *)&Y[1], &acc[1], (N/2)-1);
  acc[(N/2)-1] = X[(N/2)-1]*Y[(N/2)-1];
}

#define spectral_mul_accum16 spectral_mul_accum

/** Compute weighted cross-power spectrum of a half-complex (packed) vector with conjugate */
static inline void weighted_spectral_mul_conj(const spx_float_t *w, const spx_float_t p, const spx_word16_t *X, const spx_word16_t *Y, spx_word32_t *prod, int32_t N)
{
  int32_t i;
  int32_t j = 1;
  spx_float_t W;
  W = FLOAT_AMULT(p, w[0]);
  prod[0] = FLOAT_MUL32(W,MULT16_16(X[0],Y[0]));
  for (i=1; i<(N-1); i+=2)
  {
    W = FLOAT_AMULT(p, w[j]);
    prod[i] = FLOAT_MUL32(W,MAC16_16(MULT16_16(X[i],Y[i]), X[i+1],Y[i+1]));  //W*(X[i]*Y[i] + X[i+1]*Y[i+1])
    prod[i+1] = FLOAT_MUL32(W,MAC16_16(MULT16_16(-X[i+1],Y[i]), X[i],Y[i+1]));//W*(X[i]*Y[i+1]-X[i+1]*Y[i] )
    j++;
  }
  W = FLOAT_AMULT(p, w[j]);
  
  if (i<N)
  {
    prod[i] = FLOAT_MUL32(W,MULT16_16(X[i],Y[i]));
  }
}

static inline void mdf_adjust_prop(const spx_word32_t *W, int32_t N, int32_t M, int32_t P, spx_word16_t *prop)  //N=256 ,M=P=1
{
  UNUSED(P);
  UNUSED(M);
  spx_word16_t max_sum = 1.0f;
  spx_word32_t prop_sum = 1.0f;
  spx_word32_t tmp = 1.0f;
  arm_dot_prod_f32((float32_t *)W,(float32_t *)W,N,&tmp);
  prop[0] = spx_sqrt(tmp);
  if (prop[0] > max_sum)
  {
    max_sum = prop[0];
  }
  prop[0] += (0.1f *max_sum);
  prop_sum += prop[0];
  prop[0] = (0.99f * prop[0])/prop_sum;
}

static void adaptivestate_init_mc(SpeexEchoState *st,int32_t frame_size, int32_t filter_length, int32_t nb_mic, int32_t nb_speakers)
{
  int32_t i,N,M, C, K;
  st->K = nb_speakers;
  st->C = nb_mic;
  C=st->C;
  K=st->K;
#ifdef DUMP_ECHO_CANCEL_DATA
  if (rFile || pFile || oFile)
  {
    speex_fatal("Opening dump files twice");
  }
  rFile = fopen("aec_rec.sw", "wb");
  pFile = fopen("aec_play.sw", "wb");
  oFile = fopen("aec_out.sw", "wb");
#endif
  
  st->frame_size = frame_size;
  st->window_size = 2*frame_size;
  N = st->window_size;
  st->M = (filter_length+st->frame_size-1)/frame_size;
  M = st->M;
  st->cancel_count=0;
  st->sum_adapt = 0.0f;
  st->saturated = 0;
  st->screwed_up = 0;
  /* This is the default sampling rate */
  st->sampling_rate = 16000;
  st->spec_average = DIV32_16(SHL32(EXTEND32(st->frame_size), 15), st->sampling_rate);
#ifdef FIXED_POINT
  st->beta0 = DIV32_16(SHL32(EXTEND32(st->frame_size), 16), st->sampling_rate);
  st->beta_max = DIV32_16(SHL32(EXTEND32(st->frame_size), 14), st->sampling_rate);
#else
  st->beta0 = (2.0f*(float32_t)st->frame_size)/(float32_t)st->sampling_rate;
  st->beta_max = (.5f*(float32_t)st->frame_size)/(float32_t)st->sampling_rate;
#endif
  st->leak_estimate = 0.0f;
  
  for (i=0;i<N;i++)
  {
    st->window[i] = 0.5f-(0.5f*cosf(2.0f*M_PI*(float32_t)i/(float32_t)N));
  }
  for (i=0;i<=st->frame_size;i++)
  {
    st->power_1[i] = FLOAT_ONE;
  }
  for (i=0;i<(N*M*K*C);i++)
  {
    st->W[i] = 0.0f;
  }
  
  spx_word32_t sum = 0.0f;
  /* Ratio of ~10 between adaptation rate of first and last block */
  spx_word16_t decay = SHR32(spx_exp(NEG16(DIV32_16(QCONST16(2.4,11),M))),1);
  st->prop[0] = QCONST16(.7f, 15);
  sum = EXTEND32(st->prop[0]);
  for (i=1;i<M;i++)
  {
    st->prop[i] = MULT16_16_Q15(st->prop[i-1], decay);
    sum = ADD32(sum, EXTEND32(st->prop[i]));
  }
  for (i=M-1;i>=0;i--)
  {
    st->prop[i] = DIV32(MULT16_16(QCONST16(.8f,15), st->prop[i]),sum);
  }
  
  st->preemph = QCONST16(.9f,15);
  if (st->sampling_rate<12000)
  {
    st->notch_radius = QCONST16(.9f, 15);
  }
  else if (st->sampling_rate<24000)
  {
    st->notch_radius = QCONST16(.982f, 15);
  }
  else
  {
    st->notch_radius = QCONST16(.992f, 15);
  }
  st->adapted = 0;
  st->Pey = FLOAT_ONE;
  st->Pyy = FLOAT_ONE;
  
#ifdef TWO_PATH
  st->Davg1 = 0.0f;
  st->Davg2 = 0.0f;
  st->Dvar1 = FLOAT_ZERO;
  st->Dvar2 = FLOAT_ZERO;
#endif
}

/** Resets echo canceller state */
static void adaptivestate_reset(SpeexEchoState *st)
{
  int32_t i, M, N, C, K;
  st->cancel_count=0;
  st->screwed_up = 0;
  N = st->window_size;
  M = st->M;
  C=st->C;
  K=st->K;
  for (i=0; i<(N*M); i++)
  {
    st->W[i] = 0.0f;
  }
#ifdef TWO_PATH
  for (i=0; i<(N*M); i++)
  {
    st->foreground[i] = 0.0f;
  }
#endif
  for (i=0; i<(N*(M+1)); i++)
  {
    st->X[i] = 0.0f;
  }
  for (i=0;i<=st->frame_size;i++)
  {
    st->power[i] = 0.0f;
    st->power_1[i] = FLOAT_ONE;
    st->Eh[i] = 0.0f;
    st->Yh[i] = 0.0f;
  }
  for (i=0;i<st->frame_size;i++)
  {
    st->last_y[i] = 0.0f;
  }
  for (i=0; i<(N*C); i++)
  {
    st->E[i] = 0.0f;
  }
  for (i=0; i<(N*K); i++)
  {
    st->x[i] = 0.0f;
  }
  for (i=0; i<(2*C); i++)
  {
    st->notch_mem[i] = 0.0f;
  }
  st->memD = 0.0f;
  st->memE = 0.0f;
  st->memX = 0.0f;
  
  st->saturated = 0;
  st->adapted = 0;
  st->sum_adapt = 0.0f;
  st->Pey = FLOAT_ONE;
  st->Pyy = FLOAT_ONE;
#ifdef TWO_PATH
  st->Davg1 = 0.0f;
  st->Davg2 = 0.0f;
  st->Dvar1 = FLOAT_ZERO;
  st->Dvar2 = FLOAT_ZERO;
#endif
}

/** Performs echo cancellation on a frame */
static  void adaptive_A_run(SpeexEchoState *st, const spx_int16_t *in, const spx_int16_t *far_end, spx_int16_t *out)
{
  int32_t i,j;
  int32_t N,M, C;
  spx_word32_t Syy,See,Sxx,Sdd, Sff;
#ifdef TWO_PATH
  spx_word32_t Dbf;
  int32_t update_foreground;
#endif
  spx_word32_t Sey;
  spx_word16_t ss, ss_1;
  spx_float_t Pey = FLOAT_ONE, Pyy=FLOAT_ONE;
  spx_float_t alpha, alpha_1;
  spx_word16_t RER;
  spx_word32_t tmp32;
  
  N = 256;      /*window_size*/
  M = 1;        /*M*/
  C = 1;        /*C*/
  
  st->cancel_count++;
#ifdef FIXED_POINT
  ss=DIV32_16(11469,M);
  ss_1 = SUB16(32767,ss);
#else
  ss=.35f/(float32_t)M;
  ss_1 = 1.0f-ss;
#endif
  
  /* Apply a notch filter to make sure DC doesn't end up causing problems */
  //  filter_dc_notch16(in, st->notch_radius, st->input, ST_FRAMESIZE, st->notch_mem, C); //TO BE OPTIMIZED NOW DISABLED
  
  /* Copy input datas to buffer and apply pre-emphasis */
  for (i=0;i<ST_FRAMESIZE;i++)
  {
    spx_word32_t temp32;
    temp32= (float32_t)in[i]-(0.9f*st->memD);
    st->memD = (float32_t)in[i];
    st->input[i] = EXTRACT16(temp32);
    
    spx_word32_t tmp32_1;
    st->x[i] = st->x[i+ST_FRAMESIZE];
    tmp32_1= (float32_t)far_end[i] - (0.9f * st->memX);
    st->x[i+ST_FRAMESIZE] = tmp32_1;
    st->memX = (float32_t)far_end[i];
  }
  
  /* Shift memory: this could be optimized eventually*/ //OPTIMIZE: MEMCOPY ISN'T EFFECTIVE
  for (i=0; i<N; i++)
  {
    st->X[N+i] = st->X[i];
  }
  
  /* Convert x (echo input) to frequency domain */
  fft(st->fft_table, st->x, &st->X[0]);
  Sxx = 0.0f;
  Sxx += mdf_inner_prod(st->x+st->frame_size, st->x+st->frame_size, st->frame_size);
  power_spectrum_accum(st->X, st->Xf, N);
  Sff = 0.0f;
  
#ifdef TWO_PATH
  /* Compute foreground filter */
  spectral_mul_accum16(st->X, st->foreground, st->Y, N, M);
  ifft(st->fft_table, st->Y, st->e);
  for (i=0;i<st->frame_size;i++)
  {
    st->e[i] = SUB16(st->input[i], st->e[i+st->frame_size]);
  }
  Sff += mdf_inner_prod(st->e, st->e, st->frame_size);
#endif
  
  /* Adjust proportional adaption rate */
  if (st->adapted == 1)
  {
    mdf_adjust_prop (st->W, N, 1, 1, st->prop);
  }
  /* Compute weight gradient */
  if (st->saturated == 0)
  {
    weighted_spectral_mul_conj(st->power_1, FLOAT_SHL(PSEUDOFLOAT(st->prop[0]),-15), &st->X[N], st->E, st->PHI, N); //OPTIMIZE
    for (i=0;i<N;i++)
    {
      st->W[i] += st->PHI[i];
    }
  }
  else
  {
    st->saturated--;
  }
  
  
  ifft(st->fft_table, &st->W[0], st->wtmp);
  //IS THIS SET TO 0 NECESSARY?
  for (i=st->frame_size;i<N;i++)
  {
    st->wtmp[i]=0.0f;
  }
  
  fft(st->fft_table, st->wtmp, &st->W[0]);
  
  
  /* So we can use power_spectrum_accum */
  for (i=0;i<=st->frame_size;i++)
  {
    st->Rf[i] = 0.0f;
    st->Yf[i] = 0.0f;
    st->Xf[i] = 0.0f;
  }
  
#ifdef TWO_PATH      
  Dbf = 0.0f;
  See = 0.0f;
  /* Difference in response, this is used to estimate the variance of our residual power estimate */
  spectral_mul_accum(st->X, st->W, st->Y, N, M);
  ifft(st->fft_table, st->Y, st->y);
  for (i=0;i<st->frame_size;i++)
  {
    st->e[i] = (st->e[i+st->frame_size] - st->y[i+st->frame_size]);
  }
  Dbf += 10.0f + mdf_inner_prod(st->e, st->e, st->frame_size);
  for (i=0;i<st->frame_size;i++)
  {
    st->e[i] = (st->input[i] - st->y[i+st->frame_size]);
  }
  See += mdf_inner_prod(st->e, st->e, st->frame_size);
#endif
  
#ifndef TWO_PATH
  Sff = See;
#endif
  
#ifdef TWO_PATH
  /* Logic for updating the foreground filter */
  /* For two time windows, compute the mean of the energy difference, as well as the variance*/
  st->Davg1 = (.6f*st->Davg1) + (.4f*(Sff-See));
  st->Davg2 = (.85f*st->Davg2) + (.15f*(Sff-See));
  st->Dvar1 = (.36f*st->Dvar1) + (.16f*Sff*Dbf);
  st->Dvar2 = (.7225f*st->Dvar2) + (.0225f*Sff*Dbf);
  
  update_foreground = 0;
  /* Check if we have a statistically significant reduction in the residual echo */
  /* Note that this is *not* Gaussian, so we need to be careful about the longer tail */
  if (((Sff-See)*ABS32(Sff-See)) > (Sff * Dbf))
  {
    update_foreground = 1;
  }
  else if (FLOAT_GT(FLOAT_MUL32U(st->Davg1, ABS32(st->Davg1)), FLOAT_MULT(VAR1_UPDATE,(st->Dvar1))))
  {
    update_foreground = 1;
  }
  else if (FLOAT_GT(FLOAT_MUL32U(st->Davg2, ABS32(st->Davg2)), FLOAT_MULT(VAR2_UPDATE,(st->Dvar2))))
  {
    update_foreground = 1;
  }
  else
  {
    /**/
  }
  
  /* Do we update? */
  if (update_foreground == 1)
  {
    st->Davg1 = 0.0f;
    st->Davg2 = 0.0f;
    st->Dvar1 = FLOAT_ZERO;
    st->Dvar2 = FLOAT_ZERO;
    /* Copy background filter to foreground filter */
    for (i=0;i<N;i++)
    {
      st->foreground[i] = EXTRACT16(PSHR32(st->W[i],16));
    }
    /* Apply a smooth transition so as to not introduce blocking artifacts */
    for (i=0;i<st->frame_size;i++)
    {
      st->e[i+st->frame_size] = MULT16_16_Q15(st->window[i+st->frame_size],st->e[i+st->frame_size]) + MULT16_16_Q15(st->window[i],st->y[i+st->frame_size]);
    }
  }
  else
  {
    int32_t reset_background=0;
    /* Otherwise, check if the background filter is significantly worse */
    if (FLOAT_GT(FLOAT_MUL32U(NEG32(SUB32(Sff,See)),ABS32(SUB32(Sff,See))), FLOAT_MULT(VAR_BACKTRACK,FLOAT_MUL32U(Sff,Dbf))))
    {
      reset_background = 1;
    }
    if (FLOAT_GT(FLOAT_MUL32U(NEG32(st->Davg1), ABS32(st->Davg1)), FLOAT_MULT(VAR_BACKTRACK,st->Dvar1)))
    {
      reset_background = 1;
    }
    if (FLOAT_GT(FLOAT_MUL32U(NEG32(st->Davg2), ABS32(st->Davg2)), FLOAT_MULT(VAR_BACKTRACK,st->Dvar2)))
    {
      reset_background = 1;
    }
    if (reset_background == 1)
    {
      /* Copy foreground filter to background filter */
      for (i=0;i<N;i++)
      {
        st->W[i] = SHL32(EXTEND32(st->foreground[i]),16);
      }
      /* We also need to copy the output so as to get correct adaptation */
      for (i=0;i<st->frame_size;i++)
      {
        st->y[i+st->frame_size] = st->e[i+st->frame_size];
      }
      for (i=0;i<st->frame_size;i++)
      {
        st->e[i] = SUB16(st->input[i], st->y[i+st->frame_size]);
      }
      See = Sff;
      st->Davg1 = 0.0f;
      st->Davg2 = 0.0f;
      st->Dvar1 = FLOAT_ZERO;
      st->Dvar2 = FLOAT_ZERO;
    }
  }
#endif
  
  Sey = 0.0f;
  Syy = 0.0f;
  Sdd = 0.0f;
  /* Compute error signal (for the output with de-emphasis) */
  for (i=0;i<st->frame_size;i++)
  {
    spx_word32_t tmp_out;
#ifdef TWO_PATH
    tmp_out = SUB32(EXTEND32(st->input[i]), EXTEND32(st->e[i+st->frame_size]));
#else
    tmp_out = SUB32(EXTEND32(st->input[i]), EXTEND32(st->y[i+st->frame_size]));
#endif
    tmp_out = ADD32(tmp_out, EXTEND32(MULT16_16_P15(st->preemph, st->memE)));
    /* This is an arbitrary test for saturation in the microphone signal */
    if ((in[i*C] <= -32000) || (in[i*C] >= 32000))
    {
      if (st->saturated == 0)
      {
        st->saturated = 1;
      }
    }
    out[i*C] = (spx_int16_t)WORD2INT(tmp_out);
    st->memE = tmp_out;
  }
  
  /* Compute error signal (filter update version) */
  for (i=0;i<st->frame_size;i++)
  {
    st->e[i+st->frame_size] = st->e[i];
    st->e[i] = 0.0f;
  }
  
  /* Compute a bunch of correlations */
  /* FIXME: bad merge */
  Sey += mdf_inner_prod(st->e+st->frame_size, st->y+st->frame_size, st->frame_size);
  Syy += mdf_inner_prod(st->y+st->frame_size, st->y+st->frame_size, st->frame_size);
  Sdd += mdf_inner_prod(st->input, st->input, st->frame_size);
  
  /* Convert error to frequency domain */
  fft(st->fft_table, st->e, st->E);
  for (i=0;i<st->frame_size;i++)
  {
    st->y[i] = 0.0f;
  }
  fft(st->fft_table, st->y, st->Y);
  
  /* Compute power spectrum of echo (X), error (E) and filter response (Y) */
  power_spectrum_accum(st->E, st->Rf, N);
  power_spectrum_accum(st->Y, st->Yf, N);
  
  /* Do some sanity check */
  if (!((Syy>=0.0f) && (Sxx>=0.0f) && (See >= 0.0f)))
  {
    /* Things have gone really bad */
    st->screwed_up += 50;
    for (i=0;i<(st->frame_size*C);i++)
    {
      out[i] = 0;
    }
  }
  else if (SHR32(Sff, 2) > ADD32(Sdd, SHR32(MULT16_16(N, 10000),6)))
  {
    /* AEC seems to add lots of echo instead of removing it, let's see if it will improve */
    st->screwed_up++;
  }
  else
  {
    /* Everything's fine */
    st->screwed_up=0;
  }
  if (st->screwed_up>=50)
  {
    adaptivestate_reset(st);
    return;
  }
  
  /* Add a small noise floor to make sure not to have problems when dividing */
  See = MAX32(See, SHR32(MULT16_16(N, 100),6));
  Sxx += mdf_inner_prod(st->x+st->frame_size, st->x+st->frame_size, st->frame_size);
  power_spectrum_accum(st->X, st->Xf, N);
  /* Smooth far end energy estimate over time */
  for (j=0;j<=st->frame_size;j++)
  {
    st->power[j] = MULT16_32_Q15(ss_1,st->power[j]) + 1.0f + MULT16_32_Q15(ss,st->Xf[j]);
  }
  /* Compute filtered spectra and (cross-)correlations */
  for (j=st->frame_size;j>=0;j--)
  {
    spx_float_t Eh, Yh;
    Eh = (st->Rf[j] - st->Eh[j]);
    Yh = (st->Yf[j] - st->Yh[j]);
    Pey = FLOAT_ADD(Pey,FLOAT_MULT(Eh,Yh));
    Pyy = FLOAT_ADD(Pyy,FLOAT_MULT(Yh,Yh));
    st->Eh[j] = ((1.0f-st->spec_average)*st->Eh[j]) + (st->spec_average*st->Rf[j]);
    st->Yh[j] = ((1.0f-st->spec_average)*st->Yh[j]) + (st->spec_average*st->Yf[j]);
  }
  
  Pyy = FLOAT_SQRT(Pyy);
  Pey = FLOAT_DIVU(Pey,Pyy);
  
  /* Compute correlation updatete rate */
  tmp32 = MULT16_32_Q15(st->beta0,Syy);
  if (tmp32 > MULT16_32_Q15(st->beta_max,See))
  {
    tmp32 = MULT16_32_Q15(st->beta_max,See);
  }
  alpha = FLOAT_DIV32(tmp32, See);
  alpha_1 = FLOAT_SUB(FLOAT_ONE, alpha);
  /* Update correlations (recursive average) */
  st->Pey = FLOAT_ADD(FLOAT_MULT(alpha_1,st->Pey) , FLOAT_MULT(alpha,Pey));
  st->Pyy = FLOAT_ADD(FLOAT_MULT(alpha_1,st->Pyy) , FLOAT_MULT(alpha,Pyy));
  if (FLOAT_LT(st->Pyy, FLOAT_ONE))
  {
    st->Pyy = FLOAT_ONE;
  }
  /* We don't really hope to get better than 33 dB (MIN_LEAK-3dB) attenuation anyway */
  if (FLOAT_LT(st->Pey, FLOAT_MULT(MIN_LEAK,st->Pyy)))
  {
    st->Pey = FLOAT_MULT(MIN_LEAK,st->Pyy);
  }
  if (FLOAT_GT(st->Pey, st->Pyy))
  {
    st->Pey = st->Pyy;
  }
  /* leak_estimate is the linear regression result */
  st->leak_estimate = FLOAT_EXTRACT16(FLOAT_SHL(FLOAT_DIVU(st->Pey, st->Pyy),14));
  /* This looks like a stupid bug, but it's right (because we convert from Q14 to Q15) */
  if (st->leak_estimate > 16383.0f)
  {
    st->leak_estimate = 32767.0f;
  }
  else
  {
    st->leak_estimate = SHL16(st->leak_estimate,1);
  }
  
  /* Compute Residual to Error Ratio */
  
  RER = ((.0001f*Sxx) + (3.0f*MULT16_32_Q15(st->leak_estimate,Syy))) / See;
  /* Check for y in e (lower bound on RER) */
  if (RER < (Sey*Sey/(1.0f+(See*Syy))))
  {
    RER = Sey*Sey/(1.0f+(See*Syy));
  }
  if (RER > 0.5f)
  {
    RER = .5f;
  }
  
  
  /* We consider that the filter has had minimal adaptation if the following is true*/
  if ((st->adapted != 1) && (st->sum_adapt > SHL32(EXTEND32(M),15)) && (MULT16_32_Q15(st->leak_estimate,Syy) > MULT16_32_Q15(QCONST16(.03f,15),Syy)))
  {
    st->adapted = 1;
  }
  
  if (st->adapted == 1)
  {
    /* Normal learning rate calculation once we're past the minimal adaptation phase */
    for (i=0;i<=st->frame_size;i++)
    {
      spx_word32_t r, e;
      /* Compute frequency-domain adaptation mask */
      r = MULT16_32_Q15(st->leak_estimate,SHL32(st->Yf[i],3));
      e = SHL32(st->Rf[i],3)+1.0f;
      if (r > (0.5f*e))
      {
        r = 0.5f*e;
      }
      r = MULT16_32_Q15(QCONST16(0.7f,15),r) + MULT16_32_Q15(QCONST16(0.3f,15),(spx_word32_t)(MULT16_32_Q15(RER,e)));
      /**st->power_1[i] = adapt_rate*r/(e*(1+st->power[i]));**/
      st->power_1[i] = FLOAT_SHL(FLOAT_DIV32_FLOAT(r,FLOAT_MUL32U(e,st->power[i]+10.0f)),WEIGHT_SHIFT+16);
    }
  }
  else
  {
    /* Temporary adaption rate if filter is not yet adapted enough */
    spx_word16_t adapt_rate=0.0f;
    
    if (Sxx > SHR32(MULT16_16(N, 1000),6))
    {
      tmp32 = MULT16_32_Q15(QCONST16(.25f, 15), Sxx);
      if (tmp32 > (.25f*See))
      {
        tmp32 = .25f*See;
      }
      adapt_rate = FLOAT_EXTRACT16(FLOAT_SHL(FLOAT_DIV32(tmp32, See),15));
    }
    for (i=0;i<=st->frame_size;i++)
    {
      st->power_1[i] = FLOAT_SHL(FLOAT_DIV32(EXTEND32(adapt_rate),ADD32(st->power[i],10.0f)),WEIGHT_SHIFT+1);
    }
    /* How much have we adapted so far? */
    st->sum_adapt = ADD32(st->sum_adapt,adapt_rate);
  }
  /* FIXME: MC conversion required */
  for (i=0;i<st->frame_size;i++)
  {
    st->last_y[i] = st->last_y[st->frame_size+i];
  }
  if (st->adapted == 1)
  {
    /* If the filter is adapted, take the filtered echo */
    for (i=0;i<st->frame_size;i++)
    {
      st->last_y[st->frame_size+i] = (float32_t)in[i]-(float32_t)out[i];
    }
  }
}

/* Compute spectrum of estimated echo for use in an echo post-filter */
static void adaptiveget_residual(SpeexEchoState *st, spx_word32_t *residual_echo, int32_t len)
{
  UNUSED(len);
  int32_t i;
  spx_word16_t leak2;
  int32_t N;
  N = st->window_size;
  /* Apply hanning window (should pre-compute it)*/
  arm_mult_f32(st->window,st->last_y,st->y,(uint32_t)N);
  /* Compute power spectrum of the echo */
  fft(st->fft_table, st->y, st->Y);
  power_spectrum(st->Y, residual_echo, N);
  
  if (st->leak_estimate>0.5f)
  {
    leak2 = 1.0f;
  }
  else
  {
    leak2 = 2.0f*st->leak_estimate;
  }
  
  /* Estimate residual echo */
  for (i=0;i<=st->frame_size;i++)
  {
    residual_echo[i] = MULT16_32_Q15(leak2,residual_echo[i]);
  }
}


#endif  /*__ADAPTIVE_C*/

