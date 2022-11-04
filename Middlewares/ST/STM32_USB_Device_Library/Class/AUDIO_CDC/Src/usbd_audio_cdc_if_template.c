/**
  ******************************************************************************
  * @file    usbd_audio_cdc_if_template.c
  * @author  SRA
  * @brief   Source file for USBD CDC interface
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "usbd_audio_cdc_if_template.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

static int8_t AUDIO_CDC_Init_Audio(uint32_t  AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
static int8_t AUDIO_CDC_Init_CDC(void);
static int8_t AUDIO_CDC_DeInit(uint32_t options);
static int8_t AUDIO_CDC_Record(void);
static int8_t AUDIO_CDC_Ctrl_Audio(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t AUDIO_CDC_Stop(void);
static int8_t AUDIO_CDC_Pause(void);
static int8_t AUDIO_CDC_Resume(void);
static int8_t AUDIO_CDC_CommandMgr(uint8_t cmd);
static int8_t AUDIO_CDC_Ctrl_CDC(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t AUDIO_CDC_Receive(uint8_t* pbuf, uint32_t *Len);

USBD_AUDIO_CDC_ItfTypeDef USBD_AUDIO_CDC_fops =
{
  AUDIO_CDC_Init_Audio,
  AUDIO_CDC_Init_CDC,
  AUDIO_CDC_DeInit,
  AUDIO_CDC_Record,
  AUDIO_CDC_Ctrl_Audio,  
  AUDIO_CDC_Stop,
  AUDIO_CDC_Pause,
  AUDIO_CDC_Resume,
  AUDIO_CDC_CommandMgr,
  AUDIO_CDC_Ctrl_CDC,
  AUDIO_CDC_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_CDC_Init_CDC(void)
{
  
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_CDC_DeInit(uint32_t options)
{
  return (USBD_OK);
}


static int8_t AUDIO_CDC_Ctrl_CDC (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  return (USBD_OK);
}


static int8_t AUDIO_CDC_Receive(uint8_t* Buf, uint32_t *Len)
{
  return (USBD_OK);
}


static int8_t AUDIO_CDC_Ctrl_Audio(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  return (USBD_OK);  
}

static int8_t AUDIO_CDC_Stop(void)
{
  return (USBD_OK); 
}


static int8_t AUDIO_CDC_Pause(void)
{
  return (USBD_OK);
}


static int8_t AUDIO_CDC_Resume(void)
{
  return (USBD_OK);
}


static int8_t AUDIO_CDC_CommandMgr(uint8_t cmd)
{
  return (USBD_OK);
}

static int8_t AUDIO_CDC_Init_Audio(uint32_t  AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)
{
      return (USBD_OK);
}


static int8_t AUDIO_CDC_Record(void)
{
  return (USBD_OK);
}


/**
* @brief  Fills USB audio buffer with the right amount of data, depending on the
*			channel/frequency configuration
* @param  audioData: pointer to the PCM audio data
* @param  PCMSamples: number of PCM samples to be passed to USB engine
* @note Depending on the calling frequency, a coherent amount of samples must be passed to
*       the function. E.g.: assuming a Sampling frequency of 16 KHz and 1 channel,
*       you can pass 16 PCM samples if the function is called each millisecond,
*       32 samples if called every 2 milliseconds and so on.
*/
void Send_Audio_to_USB(int16_t * audioData, uint16_t PCMSamples){

	
}

