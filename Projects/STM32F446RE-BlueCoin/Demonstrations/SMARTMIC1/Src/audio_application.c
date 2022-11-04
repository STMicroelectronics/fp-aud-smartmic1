/**
  ******************************************************************************
  * @file    audio_application.c
  * @author  SRA
  * 
  * 
  * @brief   SMARTMIC1 application.
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

/* Includes ------------------------------------------------------------------*/
#include "audio_application.h"

#define INTERPOLATOR_BLOCK_SIZE 16
#define INTERPOLATOR_NUM_TAPS 14
#define INTERPOLATOR_FACTOR 2
#define INTERPOLATOR_STATE_LENGTH (INTERPOLATOR_BLOCK_SIZE + (INTERPOLATOR_NUM_TAPS/INTERPOLATOR_FACTOR) -1)

/** @defgroup SMARTMIC1
  * @{
  */

/** @defgroup SMARTMIC1_AUDIO
  * @{
  */


/** @defgroup SMARTMIC1_AUDIO_Private_Function_Prototypes
  * @{
  */


/**
  * @}
  */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/** @defgroup SMARTMIC1_AUDIO_Exported_Variables
  * @{
  */
/**
  * @}
  */

/** @defgroup SMARTMIC1_AUDIO_Private_Variables
  * @{
  */
BSP_AUDIO_Init_t MicParams;
BSP_AUDIO_Init_t OutParams;
/*Handler and Config structure for BeamForming*/
static AcousticBF_Handler_t  libBeamforming_Handler_Instance;
static AcousticBF_Config_t  lib_Beamforming_Config_Instance;
/*Handler and Config structure for Source Localization*/
static AcousticSL_Handler_t   libSoundSourceLoc_Handler_Instance;
static AcousticSL_Config_t    libSoundSourceLoc_Config_Instance;
/*Handler and Config structure for AEC*/
static AcousticEC_Handler_t   EchoHandlerInstance;
static AcousticEC_Config_t    EchoConfigInstance;
/*Handler and Config structure for db SPL*/
static AcousticDB_Handler_t  DB_Handler;
static AcousticDB_Config_t  DB_Config;
/*PDM to PCM and in/out related:*/
//static PDM_Filter_Handler_t FilterSTD[4];
static PDM_Filter_Handler_t FilterdB_Handler;
static PDM_Filter_Config_t FilterdB_Config;
static uint16_t aPDMBuffer[AUDIO_IN_CHANNELS * AUDIO_IN_SAMPLING_FREQUENCY / 1000 * 128 / 8];
static uint16_t aPCMBufferIN[AUDIO_IN_CHANNELS * AUDIO_IN_SAMPLING_FREQUENCY / 1000];
static uint16_t aPCMBufferOUT[AUDIO_OUT_CHANNELS * AUDIO_IN_SAMPLING_FREQUENCY / 1000];
static int16_t aAudioOutBuffer[AUDIO_OUT_BUFFER_SIZE];
/*Sampling Rate Conversion related*/
static arm_fir_interpolate_instance_q15 InterpolatorStateCH1;
static int16_t aCoeffs[] = { 1598,   2226,  -2184,  -4655,   -480,  12782,  24726,  24726,  12782, -480,  -4655,  -2184,   2226,   1598}; /*Filter coefficients of a FIR with fStop ~ Fs/2*/
static int16_t aStateCH1[INTERPOLATOR_STATE_LENGTH];
static int16_t aUpsamplerIN[INTERPOLATOR_BLOCK_SIZE];
static int16_t aUpsamplerOUT[INTERPOLATOR_BLOCK_SIZE * INTERPOLATOR_FACTOR];
/*Application Ststus*/
AudioStatus_t UserAudioStatus;
static InternalParams_t Internals;
/*Songs fragments management*/
static uint8_t SONG_INDEX = 0;
static Song_t Songs[SONG_NUMBER];

#ifdef USE_STM32F4XX_NUCLEO
static uint8_t aVolumeLin[] =
{
  254, 40, 34, 30, 28, 26, 24, 23, 22, 21, 20, 19, 18, 18, 17, 16,
  16, 15, 15, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 10,
  10, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5,
  5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0
};
#endif

void *CODEC_X_handle = NULL;

uint8_t fixed_led_mask = 0;
uint8_t toggle_led_mask = 0;
uint8_t led_off_mask = 0;
uint16_t toggle_led_timeout = 0;
uint16_t fixed_led_timeout = 0;
uint16_t ASR_led_timeout = 0;
uint8_t ASR_led_mask = 0;

/**
  * @}
  */

/** @defgroup SMARTMIC1_AUDIO_Exported_Function
  * @{
  */

/**
  * @brief  Core function that is called when 1 ms of PDM data is available.
  *     In this application all the relevant high priority audio functions are managed.
  *         The lower priority routines are called when needed.
  *     User can add his own code here to perform some additional DSP or audio analysis.
  * @param  none
  * @retval None
  */
void AudioProcess(void)
{
  int i = 0;

  /* Audio Output: I2S */
  if (!Internals.LOCK)
  {
    BSP_AUDIO_IN_PDMToPCM(BSP_AUDIO_IN_INSTANCE, (uint16_t *)aPDMBuffer, aPCMBufferIN);  /* PDM to PCM*/

    /*Fill the USB buffer in the case no algorithms are active*/
    if (!(UserAudioStatus.GeneralStatus.AlgorithmActivation & (ALGO_ACTIVATION_BF + ALGO_ACTIVATION_EC)))
    {
      memset((void *)aPCMBufferOUT, 0, ((AUDIO_IN_SAMPLING_FREQUENCY / 1000) * AUDIO_OUT_CHANNELS) * sizeof(uint16_t));

      fixed_led_mask = 0;
      if (!toggle_led_mask)
      {

        for (i = 0; i < (UserAudioStatus.GeneralStatus.Volume + 1) / 16; i++)
        {
          fixed_led_mask = (fixed_led_mask << 1) | 1;
        }
      }

      for (i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
      {
        if (UserAudioStatus.GeneralStatus.ChannelMask == (CH_MIC1 + CH_MIC2))
        {
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i] = aPCMBufferIN[AUDIO_IN_CHANNELS * i];
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i + 1] = aPCMBufferIN[AUDIO_IN_CHANNELS * i + 1];
        }
        else if (UserAudioStatus.GeneralStatus.ChannelMask == (CH_MIC1 + CH_MIC3))
        {
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i] = aPCMBufferIN[AUDIO_IN_CHANNELS * i];
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i + 1] = aPCMBufferIN[AUDIO_IN_CHANNELS * i + 2];
        }
        else if (UserAudioStatus.GeneralStatus.ChannelMask == (CH_MIC1 + CH_MIC4))
        {
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i] = aPCMBufferIN[AUDIO_IN_CHANNELS * i];
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i + 1] = aPCMBufferIN[AUDIO_IN_CHANNELS * i + 3];
        }
        else if (UserAudioStatus.GeneralStatus.ChannelMask == (CH_MIC2 + CH_MIC3))
        {
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i] = aPCMBufferIN[AUDIO_IN_CHANNELS * i + 1];
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i + 1] = aPCMBufferIN[AUDIO_IN_CHANNELS * i + 2];
        }
        else if (UserAudioStatus.GeneralStatus.ChannelMask == (CH_MIC2 + CH_MIC4))
        {
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i] = aPCMBufferIN[AUDIO_IN_CHANNELS * i + 1];
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i + 1] = aPCMBufferIN[AUDIO_IN_CHANNELS * i + 3];
        }
        else if (UserAudioStatus.GeneralStatus.ChannelMask == (CH_MIC3 + CH_MIC4))
        {
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i] = aPCMBufferIN[AUDIO_IN_CHANNELS * i + 2];
          aPCMBufferOUT[AUDIO_OUT_CHANNELS * i + 1] = aPCMBufferIN[AUDIO_IN_CHANNELS * i + 3];
        }
      }
    }
    else if (UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_BF)
    {
      if (AcousticBF_FirstStep(&((uint8_t *)(aPDMBuffer))[Internals.BF1stMicIndex],
                               &((uint8_t *)(aPDMBuffer))[Internals.BF2ndtMicIndex], (int16_t *)aPCMBufferOUT, &libBeamforming_Handler_Instance))
      {
        Internals.RunningAlgos |= ALGO_ACTIVATION_BF;
        SW_Task1_Start();  /* BF Processing Task */
      }
    }
    else if ((UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_EC) && Internals.AEC_OUT_started)
    {

      if (AcousticEC_Data_Input(aPCMBufferIN, &aAudioOutBuffer[Internals.AudioOutRdPtr], aPCMBufferOUT,
                                (AcousticEC_Handler_t *)&EchoHandlerInstance))
      {
        Internals.RunningAlgos |= ALGO_ACTIVATION_EC;
        SW_Task3_Start(); /* EC Processing Task */
      }
      Internals.AudioOutRdPtr = (Internals.AudioOutRdPtr + 64) % AUDIO_OUT_BUFFER_SIZE;

      for (i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
      {
        aPCMBufferOUT[AUDIO_OUT_CHANNELS * i + 1] = aPCMBufferIN[AUDIO_IN_CHANNELS * i];
      }
    }
    if (UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_SL)
    {
      if (AcousticSL_Data_Input((int16_t *)&aPCMBufferIN[BOTTOM_LEFT_MIC], (int16_t *)&aPCMBufferIN[TOP_RIGHT_MIC],
                                (int16_t *)&aPCMBufferIN[BOTTOM_RIGHT_MIC], (int16_t *)&aPCMBufferIN[TOP_LEFT_MIC],
                                &libSoundSourceLoc_Handler_Instance))
      {
        Internals.RunningAlgos |= ALGO_ACTIVATION_SL;
        SW_Task2_Start(); /* SL Processing Task */
      }
    }
    if (UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_DB)
    {
      int16_t aPCMBufferIN_Db[AUDIO_IN_SAMPLING_FREQUENCY / 1000];
      BSP_AUDIO_IN_PDMToPCM_DB_Noise((uint16_t *)aPDMBuffer, (uint16_t *)aPCMBufferIN_Db);

      if (AcousticDB_Data_Input(aPCMBufferIN_Db, AUDIO_IN_SAMPLING_FREQUENCY / 1000, &DB_Handler))
      {
        int32_t tempValue = 0;
        AcousticDB_Process(&tempValue, &DB_Handler);
        /*When the communication is from user to host, the user must setup the InternalStatus: TODO change in future releases*/
        UserAudioStatus.dBStatus.dBValue = (int16_t)(tempValue);
      }
    }
  }

  if (UserAudioStatus.OutputStatus.Status == AUDIOOUT_STATUS_MICS)
  {
    for (i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000; i++)
    {
      aUpsamplerIN[i] = aPCMBufferOUT[AUDIO_OUT_CHANNELS * i];
    }

    arm_fir_interpolate_q15(&InterpolatorStateCH1, aUpsamplerIN, (int16_t *)aUpsamplerOUT, INTERPOLATOR_BLOCK_SIZE);

    for (i = 0; i < AUDIO_IN_SAMPLING_FREQUENCY / 1000 * AUDIO_OUT_CHANNELS; i++)
    {
      aAudioOutBuffer[ Internals.AudioOutWrPtr] = aUpsamplerOUT[i];
      Internals.AudioOutWrPtr = (Internals.AudioOutWrPtr + 1) % AUDIO_OUT_BUFFER_SIZE;
      aAudioOutBuffer[ Internals.AudioOutWrPtr] = aUpsamplerOUT[i];
      Internals.AudioOutWrPtr = (Internals.AudioOutWrPtr + 1) % AUDIO_OUT_BUFFER_SIZE;
    }
  }

  /* Uncomment the following lines to copy the processed channel to both right and left channel (reference channel is not sent) */
//  for(i=0;i<AUDIO_IN_SAMPLING_FREQUENCY / 1000 * AUDIO_OUT_CHANNELS;i+=AUDIO_OUT_CHANNELS)
//  {
//    aPCMBufferOUT[i+1] = aPCMBufferOUT[i];
//  }

  Send_Audio_to_USB((int16_t *)aPCMBufferOUT, AUDIO_IN_SAMPLING_FREQUENCY / 1000 * AUDIO_OUT_CHANNELS);


}

/**
  * @brief  Initialize the audio devices
  * @param  None
  * @retval BSP_ERROR_NONE if no problem occurred, BSP_ERROR_NO_INIT otherwise
  */
int32_t AudioDevices_Init(void)
{
  /* Configure Audio Input peripheral - I2S*/
  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr = AUDIO_IN_CHANNELS;
  MicParams.Device = AUDIO_IN_DIGITAL_MIC;
  MicParams.SampleRate = AUDIO_IN_SAMPLING_FREQUENCY;
  MicParams.Volume = AUDIO_VOLUME_INPUT;

  /*Initialize Audio Peripherals*/
  if (BSP_AUDIO_IN_Init(BSP_AUDIO_IN_INSTANCE, &MicParams) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_NO_INIT;
  }

  /* Initialize DBNoise peripheral */
  if (BSP_AUDIO_IN_PDMToPCM_DB_Noise_Init(AUDIO_IN_SAMPLING_FREQUENCY, 4, 1) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_NO_INIT;
  }

  uint16_t volume_temp = UserAudioStatus.OutputStatus.Volume;

  /* Configure Audio Output peripheral (SAI) and external DAC */
  OutParams.BitsPerSample = 16;
  OutParams.ChannelsNbr = AUDIO_OUT_CHANNELS;
  OutParams.Device = (uint32_t)NULL;
  OutParams.SampleRate = AUDIO_OUT_SAMPLING_FREQUENCY;
  OutParams.Volume = volume_temp;

  if (BSP_AUDIO_OUT_Init(BSP_AUDIO_OUT_INSTANCE, &OutParams) != BSP_ERROR_NONE)
  {
    return BSP_ERROR_NO_INIT;
  }
  HAL_NVIC_DisableIRQ(AUDIO_OUT_INTERRUPT);
  return BSP_ERROR_NONE;
}

/**
  * @brief  Starts microphones acquisition
  * @param  None
  * @retval BSP_ERROR_NONE if no problem occurred, BSP_ERROR otherwise
  */
uint8_t StartMicAcquisition(void)
{
  /*Start audio acquisition */
  return BSP_AUDIO_IN_Record(BSP_AUDIO_IN_INSTANCE, (uint8_t *)aPDMBuffer, AUDIO_IN_BUFFER_SIZE);

}

/**
  * @brief  Initialize the User audio status with the desired algorithms settings and
  *         resets the internal variables used by the application.
  *         The user can act on these parameters in order to fine-tune the application
  *         To be called before InitializeApplication() function
  * @param  None
  * @retval None
  */
void AudioStatus_Init(void)
{
  Internals.AudioOutStatus = AUDIOOUT_STATUS_STOP;
  Internals.BF1stMicIndex = 0;
  Internals.BF2ndtMicIndex = 0;
  Internals.AudioOutRdPtr = 0;
  Internals.AudioOutWrPtr = 0;
  Internals.RunningAlgos = 0;
  Internals.LOCK = 0;

  UserAudioStatus.GeneralStatus.AvailableModules = ALGO_ACTIVATION_BF | ALGO_ACTIVATION_SL | ALGO_ACTIVATION_EC | ALGO_ACTIVATION_DB;
  UserAudioStatus.GeneralStatus.AlgorithmActivation =  ALGO_ACTIVATION_BF | ALGO_ACTIVATION_SL | ALGO_ACTIVATION_DB;

  UserAudioStatus.GeneralStatus.ChannelMask = CH_MIC1 + CH_MIC2;
  UserAudioStatus.GeneralStatus.ChannelNumber = 2;
  UserAudioStatus.GeneralStatus.Volume = AUDIO_VOLUME_INPUT;
  UserAudioStatus.GeneralStatus.SamplingFreq = 16000;
  UserAudioStatus.BeamStatus.Direction = 5;
  UserAudioStatus.BeamStatus.Gain = 0;
  UserAudioStatus.BeamStatus.Type = ACOUSTIC_BF_TYPE_STRONG;

  UserAudioStatus.SLocStatus.Algorithm = ACOUSTIC_SL_ALGORITHM_GCCP;
  UserAudioStatus.SLocStatus.Angle = ACOUSTIC_SL_NO_AUDIO_DETECTED;
  UserAudioStatus.SLocStatus.Resolution = 10;
  UserAudioStatus.SLocStatus.Threshold = 50;

  UserAudioStatus.AECStatus.AGCValue = 0;
  UserAudioStatus.AECStatus.Denoiser = ACOUSTIC_EC_PREPROCESS_ENABLE;
  UserAudioStatus.AECStatus.TailLength = 512;

  UserAudioStatus.ASRStatus.RecognizedWord = 0;

  UserAudioStatus.dBStatus.offset = 0;
  UserAudioStatus.dBStatus.dBValue = 0;

  UserAudioStatus.OutputStatus.Status = AUDIOOUT_STATUS_MICS;
  UserAudioStatus.OutputStatus.Volume = AUDIO_VOLUME_OUTPUT;

  AudioStatus_t *InternalStatus = GetAudioStatusInternal();
  memcpy(InternalStatus, &UserAudioStatus, sizeof(AudioStatus_t));
}

/**
  * @brief  Initialize the application depending on the current audio status
  *         Includes:
  *         - Memory allocation/deallocation
  *         - Acoustic libraries init/deinit
  *         - Output setup
  * @param  None
  * @retval None
  */
uint32_t InitializeApplication(void)
{
  fixed_led_mask = 0;
  toggle_led_mask = 0;
  /*Initialize Interpolator*/
  arm_fir_interpolate_init_q15((arm_fir_interpolate_instance_q15 *) &InterpolatorStateCH1, INTERPOLATOR_FACTOR,
                               INTERPOLATOR_NUM_TAPS, aCoeffs, aStateCH1, INTERPOLATOR_BLOCK_SIZE);

  if ((UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_EC)
      && (UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_BF))
  {
    while (1); /*error if EC and BF are activated at the same time*/
  }

  BSP_AUDIO_IN_SetVolume(BSP_AUDIO_IN_INSTANCE, (uint32_t)UserAudioStatus.GeneralStatus.Volume);

  SW_IRQ_Tasks_Disable();

  /*wait for the running algos to end*/
  while (Internals.RunningAlgos != 0);

  /* Enable CRC peripheral to unlock the libraries */
  __CRC_CLK_ENABLE();

  EC_DeInit();
  SL_DeInit();
  BF_DeInit();

  if (UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_BF)
  {
    BF_Init();
    UserAudioStatus.OutputStatus.Status = AUDIOOUT_STATUS_MICS;
  }

  if (UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_SL)
  {
    SL_Init();
    UserAudioStatus.OutputStatus.Status = AUDIOOUT_STATUS_MICS;
  }

  if (UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_EC)
  {
    EC_Init();
    UserAudioStatus.OutputStatus.Status = AUDIOOUT_STATUS_SONG;
    fixed_led_mask = 0xAA;
  }
  else
  {
    UserAudioStatus.OutputStatus.Status = AUDIOOUT_STATUS_MICS;
  }

  if (UserAudioStatus.GeneralStatus.AlgorithmActivation & ALGO_ACTIVATION_DB)
  {
    DB_Init();
  }

  /*Stop Audio Out if playing*/
  if (Internals.AudioOutStatus != (AUDIOOUT_STATUS_STOP || AUDIOOUT_STATUS_PAUSE))
  {
    BSP_AUDIO_OUT_Pause(BSP_AUDIO_OUT_INSTANCE);
    HAL_NVIC_DisableIRQ(AUDIO_OUT_INTERRUPT);
    Internals.AudioOutRdPtr = 0;
    Internals.AudioOutWrPtr = 0;//(AUDIO_OUT_BUFFER_SIZE / 4);
    Internals.AudioOutStatus = AUDIOOUT_STATUS_PAUSE;
    Songs[0].out_ptr = 0;
    Songs[1].out_ptr = 0;
  }
  if (UserAudioStatus.OutputStatus.Status == AUDIOOUT_STATUS_SONG
      || UserAudioStatus.OutputStatus.Status == AUDIOOUT_STATUS_MICS)
  {
    if (UserAudioStatus.OutputStatus.Status == AUDIOOUT_STATUS_SONG)
    {
      HAL_NVIC_EnableIRQ(AUDIO_OUT_INTERRUPT);
    }
    if (Internals.AudioOutStatus == AUDIOOUT_STATUS_STOP)
    {
      BSP_AUDIO_OUT_Play(BSP_AUDIO_OUT_INSTANCE, (uint8_t *)aAudioOutBuffer, AUDIO_OUT_BUFFER_SIZE);
    }
    else if (Internals.AudioOutStatus == AUDIOOUT_STATUS_PAUSE)
    {
      BSP_AUDIO_OUT_Resume(BSP_AUDIO_OUT_INSTANCE);
    }
  }

  Internals.AudioOutStatus = UserAudioStatus.OutputStatus.Status;
  SW_IRQ_Tasks_Enable();
  return 0;
}

/**
  * @brief  This function must be called periodically in order to manage the communication with the host.
  *         Includes:
  *         - Check for any modification in the InternalStatus respect to the UserStatus, that may be caused caused by host requests
  *         - User takes actions in respons of the requests
  *         - Re-align UserStatus with InternalStatus
  * @param  None
  * @retval None
  */
void PeriodicAudioStatusManager(void)
{
  AudioStatus_t *InternalStatus = GetAudioStatusInternal();
  uint8_t AudioStatusLastChanged = GetAudioStatusLastChanged();

  /* Device to Host data */
  InternalStatus->SLocStatus.Angle = UserAudioStatus.SLocStatus.Angle;
  InternalStatus->dBStatus.dBValue = UserAudioStatus.dBStatus.dBValue;

  /*nothing is changed, no requests from host*/
  if (AudioStatusLastChanged == 0)
  {
    return;
  }

  /*host has requested modification of the general configuration*/
  if (AudioStatusLastChanged & DOMAIN_GENERAL)
  {
    /*actions*/
    if (UserAudioStatus.GeneralStatus.Volume != InternalStatus->GeneralStatus.Volume)
    {
      UserAudioStatus.GeneralStatus.Volume = InternalStatus->GeneralStatus.Volume;
      BSP_AUDIO_IN_SetVolume(BSP_AUDIO_IN_INSTANCE, (uint32_t)InternalStatus->GeneralStatus.Volume);
    }

    if (UserAudioStatus.GeneralStatus.ChannelMask != InternalStatus->GeneralStatus.ChannelMask)
    {
      UserAudioStatus.GeneralStatus.ChannelMask = InternalStatus->GeneralStatus.ChannelMask;
    }

    if (UserAudioStatus.GeneralStatus.Reserved[0] != InternalStatus->GeneralStatus.Reserved[0])
    {
      UserAudioStatus.GeneralStatus.Reserved[0] = InternalStatus->GeneralStatus.Reserved[0];
    }

    /*The lock is needed because memory will be reallocated, thus the processing must be stopped*/
    /*Wait for the lock to be released*/
    while (Internals.LOCK);
    Internals.LOCK = 1;
    if (UserAudioStatus.GeneralStatus.AlgorithmActivation != InternalStatus->GeneralStatus.AlgorithmActivation)
    {
      /*copy*/
      memcpy(&(UserAudioStatus.GeneralStatus), &(InternalStatus->GeneralStatus), sizeof(GeneralParam_t));
      /*action: reinitialize the whole application depending on the new algorithm that are active*/
      InitializeApplication();
    }
    Internals.LOCK = 0;
  }

  /*host has requested modification of the SL configuration*/
  if (AudioStatusLastChanged & DOMAIN_SLOC)
  {
    /*actions*/
    if (UserAudioStatus.SLocStatus.Resolution != InternalStatus->SLocStatus.Resolution
        || UserAudioStatus.SLocStatus.Threshold != InternalStatus->SLocStatus.Threshold)
    {
      libSoundSourceLoc_Config_Instance.resolution = InternalStatus->SLocStatus.Resolution;
      libSoundSourceLoc_Config_Instance.threshold = InternalStatus->SLocStatus.Threshold;
      AcousticSL_setConfig(&libSoundSourceLoc_Handler_Instance, &libSoundSourceLoc_Config_Instance);
    }
    /*copy*/
    memcpy(&(UserAudioStatus.SLocStatus), &(InternalStatus->SLocStatus), sizeof(SLocParam_t));
  }

  /*host has requested modification of the BF configuration*/
  if (AudioStatusLastChanged & DOMAIN_BEAMFORMING)
  {
    /*actions*/
    if (UserAudioStatus.BeamStatus.Direction != InternalStatus->BeamStatus.Direction)
    {
      BeamDirectionSetup(InternalStatus->BeamStatus.Direction);
    }
    if (UserAudioStatus.BeamStatus.Type != InternalStatus->BeamStatus.Type)
    {
      lib_Beamforming_Config_Instance.algorithm_type = InternalStatus->BeamStatus.Type;
      AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance);
    }
    /*copy*/
    memcpy(&(UserAudioStatus.BeamStatus), &(InternalStatus->BeamStatus), sizeof(BeamParam_t));
  }

  /*host has requested modification of the EC configuration*/
  if (AudioStatusLastChanged & DOMAIN_AEC)
  {
    /* actions*/
    if (UserAudioStatus.AECStatus.Denoiser != InternalStatus->AECStatus.Denoiser
        || UserAudioStatus.AECStatus.AGCValue != InternalStatus->AECStatus.AGCValue)
    {
      EchoConfigInstance.preprocess_state = InternalStatus->AECStatus.Denoiser;
      EchoConfigInstance.AGC_value = InternalStatus->AECStatus.AGCValue;
      AcousticEC_setConfig((AcousticEC_Handler_t *)&EchoHandlerInstance, (AcousticEC_Config_t *) &EchoConfigInstance);
    }
    /*copy*/
    memcpy(&(UserAudioStatus.AECStatus), &(InternalStatus->AECStatus), sizeof(AECParam_t));
  }

  /*host has requested modification of the DB configuration*/
  if (AudioStatusLastChanged & DOMAIN_DBNOISE)
  {
    /* actions*/
    /*copy*/
    memcpy(&(UserAudioStatus.dBStatus), &(InternalStatus->dBStatus), sizeof(dBParam_t));
  }

  /*host has requested modification of the ASR configuration*/
  if (AudioStatusLastChanged & DOMAIN_ASR)
  {
    /* actions*/
    /*copy*/
    memcpy(&(UserAudioStatus.ASRStatus), &(InternalStatus->ASRStatus), sizeof(ASRParam_t));
  }

  /*host has requested modification of the Output configuration*/
  if (AudioStatusLastChanged & DOMAIN_OUTPUT)
  {
    uint16_t volume_temp = InternalStatus->OutputStatus.Volume ;
    BSP_AUDIO_OUT_SetVolume(BSP_AUDIO_OUT_INSTANCE, volume_temp);
    /*copy*/
    UserAudioStatus.OutputStatus.Volume = InternalStatus->OutputStatus.Volume;
  }
  ResetAudioStatusLastChanged();
}



/**
  * @brief  Init beamforming library
  * @param  None
  * @retval None
  */
uint32_t BF_Init(void)
{
  uint32_t error_value = 0;

  libBeamforming_Handler_Instance.algorithm_type_init = ACOUSTIC_BF_TYPE_STRONG;
  libBeamforming_Handler_Instance.ref_mic_enable = ACOUSTIC_BF_REF_ENABLE;
  libBeamforming_Handler_Instance.ptr_out_channels = 2;
  libBeamforming_Handler_Instance.data_format = ACOUSTIC_BF_DATA_FORMAT_PDM;
  libBeamforming_Handler_Instance.sampling_frequency = 2048;
  libBeamforming_Handler_Instance.ptr_M1_channels = 4;
  libBeamforming_Handler_Instance.ptr_M2_channels = 4;
  libBeamforming_Handler_Instance.delay_enable = 1;
  AcousticBF_getMemorySize(&libBeamforming_Handler_Instance);
  libBeamforming_Handler_Instance.pInternalMemory = (uint32_t *)malloc(libBeamforming_Handler_Instance.internal_memory_size);

  if (libBeamforming_Handler_Instance.pInternalMemory == NULL)
  {
    while (1); /*Error Management*/
  }

  error_value = AcousticBF_Init(&libBeamforming_Handler_Instance);

  /*Error Management*/
  if (error_value != 0)
  {
    while (1); /*Error Management*/
  }

  /*Setup Beamforming dynamic parameters*/
  lib_Beamforming_Config_Instance.algorithm_type = UserAudioStatus.BeamStatus.Type;
  lib_Beamforming_Config_Instance.M2_gain = UserAudioStatus.BeamStatus.Gain;
  lib_Beamforming_Config_Instance.mic_distance = SIDE_X;
  lib_Beamforming_Config_Instance.volume = 24;

  error_value = AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance);

  if (error_value != 0)
  {
    while (1);   /*Error Management*/
  }

  BeamDirectionSetup(UserAudioStatus.BeamStatus.Direction);

  return error_value;
}

/**
  * @brief  DeInit beamforming library
  * @param  None
  * @retval None
  */
void BF_DeInit(void)
{
  if (libBeamforming_Handler_Instance.pInternalMemory != NULL)
  {
    free(libBeamforming_Handler_Instance.pInternalMemory);
    libBeamforming_Handler_Instance.pInternalMemory = NULL;
  }
}

/**
  * @brief  Init source localization library
  * @param  None
  * @retval None
  */
uint32_t SL_Init(void)
{
  uint32_t error_value = 0;
  /*Setup Source Localization static parameters*/
  libSoundSourceLoc_Handler_Instance.channel_number = 4;
  libSoundSourceLoc_Handler_Instance.M12_distance = DIAGONAL;
  libSoundSourceLoc_Handler_Instance.M34_distance = DIAGONAL;
  libSoundSourceLoc_Handler_Instance.sampling_frequency = AUDIO_IN_SAMPLING_FREQUENCY;
  libSoundSourceLoc_Handler_Instance.algorithm = UserAudioStatus.SLocStatus.Algorithm;
  libSoundSourceLoc_Handler_Instance.ptr_M1_channels = 4;
  libSoundSourceLoc_Handler_Instance.ptr_M2_channels = 4;
  libSoundSourceLoc_Handler_Instance.ptr_M3_channels = 4;
  libSoundSourceLoc_Handler_Instance.ptr_M4_channels = 4;
  libSoundSourceLoc_Handler_Instance.samples_to_process = 512;
  AcousticSL_getMemorySize(&libSoundSourceLoc_Handler_Instance);
  libSoundSourceLoc_Handler_Instance.pInternalMemory = (uint32_t *)malloc(libSoundSourceLoc_Handler_Instance.internal_memory_size);

  if (libSoundSourceLoc_Handler_Instance.pInternalMemory == NULL)
  {
    while (1); /*Error Management*/
  }

  error_value = AcousticSL_Init(&libSoundSourceLoc_Handler_Instance);

  if (error_value != 0)
  {
    while (1); /*Error Management*/
  }

  /*Setup Source Localization dynamic parameters*/
  libSoundSourceLoc_Config_Instance.resolution = UserAudioStatus.SLocStatus.Resolution;
  libSoundSourceLoc_Config_Instance.threshold = UserAudioStatus.SLocStatus.Threshold;
  error_value = AcousticSL_setConfig(&libSoundSourceLoc_Handler_Instance, &libSoundSourceLoc_Config_Instance);

  if (error_value != 0)
  {
    while (1); /*Error Management*/
  }
  return error_value;

}

/**
  * @brief  DeInit SL library
  * @param  None
  * @retval None
  */
void SL_DeInit(void)
{
  if (libSoundSourceLoc_Handler_Instance.pInternalMemory != NULL)
  {
    free(libSoundSourceLoc_Handler_Instance.pInternalMemory);
    libSoundSourceLoc_Handler_Instance.pInternalMemory = NULL;
  }
}

/**
  * @brief  Init EC library
  * @param  None
  * @retval None
  */
uint32_t EC_Init(void)
{
  uint32_t error_value = 0;
  EchoHandlerInstance.tail_length = UserAudioStatus.AECStatus.TailLength;
  EchoHandlerInstance.preprocess_init = UserAudioStatus.AECStatus.Denoiser;
  EchoHandlerInstance.ptr_primary_channels = 4;
  EchoHandlerInstance.ptr_reference_channels = 4;
  EchoHandlerInstance.ptr_output_channels = 2;
  AcousticEC_getMemorySize(&EchoHandlerInstance);
  EchoHandlerInstance.pInternalMemory = (uint32_t *)malloc(EchoHandlerInstance.internal_memory_size);

  if (EchoHandlerInstance.pInternalMemory == NULL)
  {
    while (1); /*Error Management*/
  }

  error_value = AcousticEC_Init((AcousticEC_Handler_t *)&EchoHandlerInstance);

  if (error_value != 0)
  {
    while (1); /*Error Management*/
  }

  EchoConfigInstance.preprocess_state = UserAudioStatus.AECStatus.Denoiser;
  EchoConfigInstance.AGC_value = UserAudioStatus.AECStatus.AGCValue;
  EchoConfigInstance.noise_suppress_default = -15;      /* Default: -15 */
  EchoConfigInstance.echo_suppress_default = -40;       /* Default: -40 */
  EchoConfigInstance.echo_suppress_active = -15;        /* Default: -15 */
  EchoConfigInstance.residual_echo_remove = 1;          /* Default: 1   */

  error_value = AcousticEC_setConfig((AcousticEC_Handler_t *)&EchoHandlerInstance, (AcousticEC_Config_t *) &EchoConfigInstance);

  if (error_value != 0)
  {
    while (1); /*Error Management*/
  }

  Internals.AudioOutRdPtr = 0;
  return error_value;
}

/**
  * @brief  DeInit EC library
  * @param  None
  * @retval None
  */
void EC_DeInit(void)
{
  if (EchoHandlerInstance.pInternalMemory != NULL)
  {
    free(EchoHandlerInstance.pInternalMemory);
    EchoHandlerInstance.pInternalMemory = NULL;
  }
  Internals.AEC_OUT_started = 0;
}

/**
  * @brief  Init dBSPL library
  * @param  None
  * @retval None
  */
uint32_t DB_Init(void)
{
  uint32_t error_value = 0;

  DB_Handler.sampling_frequency = UserAudioStatus.GeneralStatus.SamplingFreq;
  error_value = AcousticDB_Init(&DB_Handler);

  if (error_value != 0)
  {
    while (1); /*Error Management*/
  }

  DB_Config.offset = 0;

  error_value = AcousticDB_setConfig(&DB_Handler, &DB_Config);
  if (error_value != 0)
  {
    while (1); /*Error Management*/
  }
  return error_value;
}
/**
  * @brief  Initializes four SW interrupt with different priorities
  * @param  None
  * @retval None
  */
void SW_IRQ_Tasks_Init(void)
{

  HAL_NVIC_SetPriority((IRQn_Type)EXTI1_IRQn, 0x0C, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI1_IRQn);

  HAL_NVIC_SetPriority((IRQn_Type)EXTI2_IRQn, 0x0D, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI2_IRQn);

  HAL_NVIC_SetPriority((IRQn_Type)EXTI3_IRQn, 0x0C, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI3_IRQn);

  HAL_NVIC_SetPriority((IRQn_Type)EXTI4_IRQn, 0x0B, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI4_IRQn);

}

/**
  * @brief  Enables SW interrupts
  * @param  None
  * @retval None
  */
void SW_IRQ_Tasks_Enable(void)
{
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI1_IRQn);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI2_IRQn);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI3_IRQn);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI4_IRQn);
}

/**
  * @brief  Disables SW interrupts
  * @param  None
  * @retval None
  */
void SW_IRQ_Tasks_Disable(void)
{

  HAL_NVIC_DisableIRQ((IRQn_Type)EXTI1_IRQn);
  HAL_NVIC_DisableIRQ((IRQn_Type)EXTI2_IRQn);
  HAL_NVIC_DisableIRQ((IRQn_Type)EXTI3_IRQn);
  HAL_NVIC_DisableIRQ((IRQn_Type)EXTI4_IRQn);

}

/**
  * @brief  First interrupt handler routine
  * @param  None
  * @retval None
  */
void SW_Task1_Callback(void)
{
  AcousticBF_SecondStep(&libBeamforming_Handler_Instance);
  Internals.RunningAlgos &= ~ALGO_ACTIVATION_BF;
}

/**
  * @brief  Second interrupt handler routine
  * @param  None
  * @retval None
  */
int32_t result[4];
void SW_Task2_Callback(void)
{
  AcousticSL_Process((int32_t *)result, &libSoundSourceLoc_Handler_Instance);

  if (result[0] != ACOUSTIC_SL_NO_AUDIO_DETECTED)
  {
    result[0] = (result[0] - 45);
    if (result[0] < 0)
    {
      result[0] += 360;
    }
  }
  UserAudioStatus.SLocStatus.Angle = result[0];
  Internals.RunningAlgos &= ~ALGO_ACTIVATION_SL;

  if (result[0] != ACOUSTIC_SL_NO_AUDIO_DETECTED)
  {

    if (result[0] < 23)
    {
      toggle_led_mask = LED0_MASK;
    }
    else if (result[0] < 23 + 45)
    {
      toggle_led_mask = LED45_MASK;
    }
    else if (result[0] < 23 + 45 * 2)
    {
      toggle_led_mask = LED90_MASK;
    }
    else if (result[0] < 23 + 45 * 3)
    {
      toggle_led_mask = LED135_MASK;
    }
    else if (result[0] < 23 + 45 * 4)
    {
      toggle_led_mask = LED180_MASK;
    }
    else if (result[0] < 23 + 45 * 5)
    {
      toggle_led_mask = LED225_MASK;
    }
    else if (result[0] < 23 + 45 * 6)
    {
      toggle_led_mask = LED270_MASK;
    }
    else if (result[0] < 23 + 45 * 7)
    {
      toggle_led_mask = LED315_MASK;
    }
    else if (result[0] < 360)
    {
      toggle_led_mask = LED0_MASK;
    }

    toggle_led_timeout = 500;
  }
}

/**
  * @brief  Third interrupt handler routine
  * @param  None
  * @retval None
  */
void SW_Task3_Callback(void)
{

  AcousticEC_Process((AcousticEC_Handler_t *)&EchoHandlerInstance);
  Internals.RunningAlgos &= ~ALGO_ACTIVATION_EC;

}

/**
  * @brief  Fourth interrupt handler routine
  * @param  None
  * @retval None
  */
void SW_Task4_Callback(void)
{
}

/**
  * @brief Throws first SW interrupt
  * @param  None
  * @retval None
  */
void SW_Task1_Start(void)
{
  HAL_NVIC_SetPendingIRQ(EXTI1_IRQn);
}

/**
  * @brief Throws second SW interrupt
  * @param  None
  * @retval None
  */
void SW_Task2_Start(void)
{
  HAL_NVIC_SetPendingIRQ(EXTI2_IRQn);
}

/**
  * @brief Throws third SW interrupt
  * @param  None
  * @retval None
  */
void SW_Task3_Start(void)
{
  HAL_NVIC_SetPendingIRQ(EXTI3_IRQn);
}

/**
  * @brief Throws fourth interrupt
  * @param  None
  * @retval None
  */
void SW_Task4_Start(void)
{
  HAL_NVIC_SetPendingIRQ(EXTI4_IRQn);
}

/**
  * @brief  Half Transfer user callback, called by BSP functions.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
  * @brief  Transfer Complete user callback, called by BSP functions.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
  * @brief  Manages Audio Out full Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  /* Generally this interrupt routine is used to load the buffer when
  a streaming scheme is used: When first Half buffer is already transferred load
  the new data to the first half of buffer while DMA is transferring data from
  the second half. And when Transfer complete occurs, load the second half of
  the buffer while the DMA is transferring from the first half ... */

  int i = 0;
  for (i = 0; i < AUDIO_OUT_BUFFER_SIZE / 8; i++)
  {
    /*Upsampling should be done here...*/
    aAudioOutBuffer[AUDIO_OUT_BUFFER_SIZE / 2 + 4 * i] = Songs[SONG_INDEX].data[Songs[SONG_INDEX].out_ptr];
    aAudioOutBuffer[AUDIO_OUT_BUFFER_SIZE / 2 + 4 * i + 1] = Songs[SONG_INDEX].data[Songs[SONG_INDEX].out_ptr];
    aAudioOutBuffer[AUDIO_OUT_BUFFER_SIZE / 2 + 4 * i + 2] = Songs[SONG_INDEX].data[Songs[SONG_INDEX].out_ptr];
    aAudioOutBuffer[AUDIO_OUT_BUFFER_SIZE / 2 + 4 * i + 3] = Songs[SONG_INDEX].data[Songs[SONG_INDEX].out_ptr];
    Songs[SONG_INDEX].out_ptr = (Songs[SONG_INDEX].out_ptr + 1) % Songs[SONG_INDEX].length;
  }

  Internals.AudioOutWrPtr = 0;
}

/**
  * @brief  Manages Audio Out DMA half transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Generally this interrupt routine is used to load the buffer when
  a streaming scheme is used: When first Half buffer is already transferred load
  the new data to the first half of buffer while DMA is transferring data from
  the second half. And when Transfer complete occurs, load the second half of
  the buffer while the DMA is transferring from the first half ... */

  int i = 0;
  for (i = 0; i < AUDIO_OUT_BUFFER_SIZE / 8; i++)
  {
    /*Upsampling should be done here...*/
    aAudioOutBuffer[4 * i] = Songs[SONG_INDEX].data[Songs[SONG_INDEX].out_ptr];
    aAudioOutBuffer[4 * i + 1] = Songs[SONG_INDEX].data[Songs[SONG_INDEX].out_ptr];
    aAudioOutBuffer[4 * i + 2] = Songs[SONG_INDEX].data[Songs[SONG_INDEX].out_ptr];
    aAudioOutBuffer[4 * i + 3] = Songs[SONG_INDEX].data[Songs[SONG_INDEX].out_ptr];
    Songs[SONG_INDEX].out_ptr = (Songs[SONG_INDEX].out_ptr + 1) % Songs[SONG_INDEX].length;
  }
  Internals.AudioOutWrPtr = AUDIO_OUT_BUFFER_SIZE / 2;
  if (Internals.AEC_OUT_started == 0)
  {
    Internals.AEC_OUT_started = 1;
  }
}



/**
  * @brief Initialize song data structures, linking them to the audio fragments
  *        stored in flash
  * @param  None
  * @retval None
  */
void Songs_Init(void)
{

#if (SONG_NUMBER >= 1)
  Songs[0].data = (int16_t *)Fragment1;
  Songs[0].length = Fragment1_size;
  Songs[0].in_ptr = 0;
  Songs[0].out_ptr = 0;
#endif

#if (SONG_NUMBER >= 2)
  Songs[1].data = (int16_t *)Fragment2;
  Songs[1].length = Fragment2_size;
  Songs[1].in_ptr = 0;
  Songs[1].out_ptr = 0;
#endif

#if (SONG_NUMBER >= 3)
  Songs[2].data = (int16_t *)Fragment3;
  Songs[2].length = Fragment3_size;
  Songs[2].in_ptr = 0;
  Songs[2].out_ptr = 0;
#endif
}


/**
  * @brief  Init to converts audio format from PDM to PCM for the dB noise use.
  * @param  PDMBuf: Pointer to PDM buffer data
  * @param  PCMBuf: Pointer to PCM buffer data
  * @retval BSP_ERROR_NONE in case of success, BSP_ERROR otherwise
  */

int32_t BSP_AUDIO_IN_PDMToPCM_DB_Noise_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{
  volatile uint32_t error = 0;
  /* Init PDM filters */
  FilterdB_Handler.bit_order  = PDM_FILTER_BIT_ORDER_LSB;
  FilterdB_Handler.endianness = PDM_FILTER_ENDIANNESS_LE;
  FilterdB_Handler.high_pass_tap = 2122358088;
  FilterdB_Handler.out_ptr_channels = ChnlNbrOut;
  FilterdB_Handler.in_ptr_channels  = ChnlNbrIn;
  error = PDM_Filter_Init((PDM_Filter_Handler_t *)(&FilterdB_Handler));

  if (error != 0)
  {
    while (1);
  }

  /* PDM lib config phase */
  FilterdB_Config.output_samples_number = AudioFreq / 1000;
  FilterdB_Config.mic_gain = 24;

  uint32_t DecimationFactor = PDM_FREQ_16K * 1000 / AudioFreq;

  switch (DecimationFactor)
  {
    case 16:
      FilterdB_Config.decimation_factor = PDM_FILTER_DEC_FACTOR_16;
      break;
    case 24:
      FilterdB_Config.decimation_factor = PDM_FILTER_DEC_FACTOR_24;
      break;
    case 32:
      FilterdB_Config.decimation_factor = PDM_FILTER_DEC_FACTOR_32;
      break;
    case 48:
      FilterdB_Config.decimation_factor = PDM_FILTER_DEC_FACTOR_48;
      break;
    case 64:
      FilterdB_Config.decimation_factor = PDM_FILTER_DEC_FACTOR_64;
      break;
    case 80:
      FilterdB_Config.decimation_factor = PDM_FILTER_DEC_FACTOR_80;
      break;
    case 128:
      FilterdB_Config.decimation_factor = PDM_FILTER_DEC_FACTOR_128;
      break;
    case 160:
      FilterdB_Config.decimation_factor = PDM_FILTER_DEC_FACTOR_80;
      break;
    default:
      return BSP_ERROR_WRONG_PARAM;
  }
  error = PDM_Filter_setConfig((PDM_Filter_Handler_t *)&FilterdB_Handler, &FilterdB_Config);

  if (error != 0)
  {
    while (1);
  }


  return BSP_ERROR_NONE;
}

/**
  * @brief  Converts audio format from PDM to PCM for the dB noise use.
  * @param  PDMBuf: Pointer to PDM buffer data
  * @param  PCMBuf: Pointer to PCM buffer data
  * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
  */

uint8_t BSP_AUDIO_IN_PDMToPCM_DB_Noise(uint16_t *PDMBuf, uint16_t *PCMBuf)
{

  PDM_Filter(((uint8_t *)(PDMBuf)), (uint16_t *)(PCMBuf), (PDM_Filter_Handler_t *)&FilterdB_Handler);
  return BSP_ERROR_NONE;
}


/** @defgroup SMARTMIC1_AUDIO_Private_Function_Prototypes
  * @{
  */

/**
  * @brief  Support function needed to setup the correct beamforming microphones depending
  *         on the direction choosen by the user.
  * @param  direction: parameter that describes the desired output:
  *         1 to 8 for one of the possible beamforming directions*
  * @retval None
  */
void BeamDirectionSetup(uint8_t direction)
{
  switch (direction)
  {
    case 1:
      Internals.BF1stMicIndex = TOP_RIGHT_MIC;
      Internals.BF2ndtMicIndex = BOTTOM_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = SIDE_Y;
      fixed_led_mask = LED0_MASK;
      break;
    case 2:
      Internals.BF1stMicIndex = TOP_RIGHT_MIC;
      Internals.BF2ndtMicIndex = BOTTOM_LEFT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
      fixed_led_mask = LED45_MASK;
      break;
    case 3:
      Internals.BF1stMicIndex = BOTTOM_RIGHT_MIC;
      Internals.BF2ndtMicIndex = BOTTOM_LEFT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = SIDE_X;
      fixed_led_mask = LED90_MASK;
      break;
    case 4:
      Internals.BF1stMicIndex = BOTTOM_RIGHT_MIC;
      Internals.BF2ndtMicIndex = TOP_LEFT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
      fixed_led_mask = LED135_MASK;
      break;
    case 5:
      Internals.BF1stMicIndex = BOTTOM_RIGHT_MIC;
      Internals.BF2ndtMicIndex = TOP_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = SIDE_Y;
      fixed_led_mask = LED180_MASK;
      break;
    case 6:
      Internals.BF1stMicIndex = BOTTOM_LEFT_MIC;
      Internals.BF2ndtMicIndex = TOP_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
      fixed_led_mask = LED225_MASK;
      break;
    case 7:
      Internals.BF1stMicIndex = BOTTOM_LEFT_MIC;
      Internals.BF2ndtMicIndex = BOTTOM_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = SIDE_X;
      fixed_led_mask = LED270_MASK;
      break;
    case 8:
      Internals.BF1stMicIndex = TOP_LEFT_MIC;
      Internals.BF2ndtMicIndex = BOTTOM_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
      fixed_led_mask = LED315_MASK;
      break;
  }
  LedsOn(fixed_led_mask);
  AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance);
}


void LedsToggle(uint8_t LedMask)
{
  if (LedMask & LED0_MASK) { BSP_LED_Toggle(LED1); }
  if (LedMask & LED45_MASK) { BSP_LED_Toggle(LED2); }
  if (LedMask & LED90_MASK) { BSP_LED_Toggle(LED3); }
  if (LedMask & LED135_MASK) { BSP_LED_Toggle(LED4); }
  if (LedMask & LED180_MASK) { BSP_LED_Toggle(LED5); }
  if (LedMask & LED225_MASK) { BSP_LED_Toggle(LED6); }
  if (LedMask & LED270_MASK) { BSP_LED_Toggle(LED7); }
  if (LedMask & LED315_MASK) { BSP_LED_Toggle(LED8); }
}

void LedsOn(uint8_t LedMask)
{
  if (LedMask & LED0_MASK) { BSP_LED_On(LED1); }
  if (LedMask & LED45_MASK) { BSP_LED_On(LED2); }
  if (LedMask & LED90_MASK) { BSP_LED_On(LED3); }
  if (LedMask & LED135_MASK) { BSP_LED_On(LED4); }
  if (LedMask & LED180_MASK) { BSP_LED_On(LED5); }
  if (LedMask & LED225_MASK) { BSP_LED_On(LED6); }
  if (LedMask & LED270_MASK) { BSP_LED_On(LED7); }
  if (LedMask & LED315_MASK) { BSP_LED_On(LED8); }
}

void LedsOff(uint8_t LedMask)
{
  if (LedMask & LED0_MASK) { BSP_LED_Off(LED1); }
  if (LedMask & LED45_MASK) { BSP_LED_Off(LED2); }
  if (LedMask & LED90_MASK) { BSP_LED_Off(LED3); }
  if (LedMask & LED135_MASK) { BSP_LED_Off(LED4); }
  if (LedMask & LED180_MASK) { BSP_LED_Off(LED5); }
  if (LedMask & LED225_MASK) { BSP_LED_Off(LED6); }
  if (LedMask & LED270_MASK) { BSP_LED_Off(LED7); }
  if (LedMask & LED315_MASK) { BSP_LED_Off(LED8); }
}


HAL_StatusTypeDef MX_SAI1_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t SampleRate)
{
  return HAL_OK;
}

HAL_StatusTypeDef MX_I2S_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t PDM_rate)
{
  return HAL_OK;
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


