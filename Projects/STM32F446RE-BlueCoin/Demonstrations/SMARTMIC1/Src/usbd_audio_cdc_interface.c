/**
  ******************************************************************************
  * @file    usbd_cdc_interface.c
  * @author  SRA
  * 
  * 
  * @brief   Source file for USBD CDC interface
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

#include "usbd_audio_cdc_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define APP_RX_DATA_SIZE  512
#define APP_TX_DATA_SIZE  512

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */

uint32_t UserTxBufPtrIn = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
uint32_t UserTxBufPtrOut = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */


volatile uint8_t USB_RxBuffer[USB_RxBufferDim];
volatile uint16_t USB_RxBufferStart_idx = 0;

/* TIM handler declaration */
TIM_HandleTypeDef  TimHandle;
/* USB handler declaration */
extern USBD_HandleTypeDef  hUSBDDevice;
extern BSP_AUDIO_Init_t MicParams;

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(void);
static void TIM_Config(void);
static int8_t AUDIO_CDC_Init_Audio(uint32_t  AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
static int8_t AUDIO_CDC_Init_CDC(void);
static int8_t AUDIO_CDC_DeInit(uint32_t options);
static int8_t AUDIO_CDC_Record(void);
static int8_t AUDIO_CDC_Ctrl_Audio(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t AUDIO_CDC_Stop(void);
static int8_t AUDIO_CDC_Pause(void);
static int8_t AUDIO_CDC_Resume(void);
static int8_t AUDIO_CDC_CommandMgr(uint8_t cmd);
static int8_t AUDIO_CDC_Ctrl_CDC(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t AUDIO_CDC_Receive(uint8_t *pbuf, uint32_t *Len);

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
  /*##-2- Enable TIM peripherals Clock #######################################*/
  TIMx_CLK_ENABLE();

  /*##-3- Configure the NVIC for TIMx ########################################*/
  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriority(TIMx_IRQn, 0x6, 0);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);
  /*##-4- Configure the TIM Base generation  #################################*/
  TIM_Config();

  /*##-5- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /*##-6- Set Application Buffers ############################################*/
  USBD_CDC_SetTxBuffer(&hUSBDDevice, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&hUSBDDevice, UserRxBuffer);

  return (USBD_OK);
}

/**
  * @brief  Fill the usb tx buffer
  * @param  Buf: pointer to the tx buffer
  * @param  TotalLen: number of bytes to be sent
  * @retval Result of the operation: USBD_OK if all operations are OK
  */
uint8_t CDC_Fill_Buffer(uint8_t *Buf, uint32_t TotalLen)
{
  uint16_t i;

  for (i = 0; i < TotalLen; i++)
  {
    UserTxBuffer[UserTxBufPtrIn] = Buf[i];
    UserTxBufPtrIn = (UserTxBufPtrIn + 1) % APP_RX_DATA_SIZE;
  }
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

/**
  * @brief  Controls AUDIO Volume.
  * @param  vol: Volume level (0..100)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
volatile uint8_t VolumeSetting = 64;
const int16_t vol_table[65] =
{
  0x8000, 0xDBE0, 0xE1E6, 0xE56B, 0xE7EB, 0xE9DB, 0xEB70, 0xECC7,
  0xEDF0, 0xEEF6, 0xEFE0, 0xF0B4, 0xF176, 0xF228, 0xF2CD, 0xF366,
  0xF3F5, 0xF47C, 0xF4FB, 0xF574, 0xF5E6, 0xF652, 0xF6BA, 0xF71C,
  0xF778, 0xF7D6, 0xF82D, 0xF881, 0xF8D2, 0xF920, 0xF96B, 0xF9B4,
  0xF9FB, 0xFA3F, 0xFA82, 0xFAC2, 0xFB01, 0xFB3E, 0xFB79, 0xFBB3,
  0xFBEB, 0xFC22, 0xFC57, 0xFC8C, 0xFCBF, 0xFCF1, 0xFD22, 0xFD51,
  0xFD80, 0xFDAE, 0xFDDB, 0xFE07, 0xFE32, 0xFE5D, 0xFE86, 0xFEAF,
  0xFED7, 0xFF00, 0xFF25, 0xFF4B, 0xFF70, 0xFF95, 0xFFB9, 0xFFD0,
  0x0000
};


int8_t Audio_VolumeCtl(int16_t Volume)
{
  /* Call low layer volume setting function */
  int j;

  j = 0;
  /* Find the setting nearest to the desired setting */
  while (j < 64 &&
         abs(Volume - vol_table[j]) > abs(Volume - vol_table[j + 1]))
  {
    j++;
  }
  /* Now do the volume adjustment */

  /*USB Volume control is disabled in this demo
  VolumeSetting = (uint8_t)j;
  return  BSP_AUDIO_IN_SetVolume(j); */

  return BSP_ERROR_NONE;
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_CDC_Ctrl_CDC(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

      // Audio_VolumeCtl(*((int16_t *)(pbuf)));
      /* Add your code here */
      break;

    case CDC_SET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_GET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_CLEAR_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_SET_LINE_CODING:
      LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | \
                                         (pbuf[2] << 16) | (pbuf[3] << 24));
      LineCoding.format     = pbuf[4];
      LineCoding.paritytype = pbuf[5];
      LineCoding.datatype   = pbuf[6];

      /* Set the new configuration */
      //  ComPort_Config();
      break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(LineCoding.bitrate);
      pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
      pbuf[4] = LineCoding.format;
      pbuf[5] = LineCoding.paritytype;
      pbuf[6] = LineCoding.datatype;
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      /* Add your code here */
      break;

    case CDC_SEND_BREAK:
      /* Add your code here */
      break;


    case AUDIO_REQ_GET_CUR:
      /* Add your code here */
      break;

    case AUDIO_REQ_GET_MIN:
      /* Add your code here */
      break;

    case AUDIO_REQ_GET_MAX:
      /* Add your code here */
      break;

    case AUDIO_REQ_GET_RES:
      /* Add your code here */
      break;


    default:
      break;
  }
  return (USBD_OK);
}

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t buffptr;
  uint32_t buffsize;

  if (UserTxBufPtrOut != UserTxBufPtrIn)
  {
    if (UserTxBufPtrOut > UserTxBufPtrIn) /* Rollback */
    {
      buffsize = APP_RX_DATA_SIZE - UserTxBufPtrOut;
    }
    else
    {
      buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
    }

    buffptr = UserTxBufPtrOut;

    USBD_CDC_SetTxBuffer(&hUSBDDevice, (uint8_t *)&UserTxBuffer[buffptr], buffsize);

    if (USBD_CDC_TransmitPacket(&hUSBDDevice) == USBD_OK)
    {
      UserTxBufPtrOut += buffsize;
      if (UserTxBufPtrOut == APP_RX_DATA_SIZE)
      {
        UserTxBufPtrOut = 0;
      }
    }
  }
}

/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_CDC_Receive(uint8_t *Buf, uint32_t *Len)
{
  uint16_t numByteToCopy;
  if (((USB_RxBufferStart_idx) + (uint16_t)*Len) > USB_RxBufferDim)
  {
    numByteToCopy = USB_RxBufferDim - (USB_RxBufferStart_idx);
    uint16_t Buf_idx = 0;
    memcpy((uint8_t *)&USB_RxBuffer[USB_RxBufferStart_idx], (uint8_t *)&Buf[Buf_idx], numByteToCopy);
    USB_RxBufferStart_idx = 0;
    Buf_idx = numByteToCopy;
    numByteToCopy = (uint16_t)(*Len - numByteToCopy);
    memcpy((uint8_t *)&USB_RxBuffer[USB_RxBufferStart_idx], (uint8_t *)&Buf[Buf_idx], numByteToCopy);
    USB_RxBufferStart_idx = numByteToCopy;
  }
  else
  {
    numByteToCopy = (uint16_t) * Len;
    memcpy((uint8_t *)&USB_RxBuffer[USB_RxBufferStart_idx], (uint8_t *)&Buf[0], numByteToCopy);
    USB_RxBufferStart_idx = USB_RxBufferStart_idx + numByteToCopy;
    if (USB_RxBufferStart_idx == USB_RxBufferDim)
    {
      USB_RxBufferStart_idx = 0;
    }
  }

  /* Initiate next USB packet transfer */
  USBD_CDC_ReceivePacket(&hUSBDDevice);
  return (USBD_OK);
}


/**
  * @brief  Return the USB Rx buffer
  * @param  None
  * @retval USB Rx buffer pointer
  */
uint8_t *USB_GetRxBuffer(void)
{
  return (uint8_t *)USB_RxBuffer;
}

/**
  * @brief  Check if new dara are available
  * @param  None
  * @retval None
  */
uint16_t USB_CheckForNewData(void)
{
  return USB_RxBufferStart_idx;
}

/**
  * @brief  Return the USB Tx buffer
  * @param  None
  * @retval USB Tx buffer pointer
  */
uint8_t *USB_GetTxBuffer(void)
{
  return (uint8_t *)&UserTxBuffer[UserTxBufPtrIn];
}

/**
  * @brief  TIM_Config: Configure TIMx timer
  * @param  None.
  * @retval None.
  */
static void TIM_Config(void)
{
  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIM3 peripheral as follows:
       + Period = (CDC_POLLING_INTERVAL * 10000) - 1
       + Prescaler = ((APB1 frequency / 1000000) - 1)
       + ClockDivision = 0
       + Counter direction = Up  */
  TimHandle.Init.Period = (CDC_POLLING_INTERVAL * 1000) - 1;
  TimHandle.Init.Prescaler = 84 - 1;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

static int8_t AUDIO_CDC_Ctrl_Audio(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd)
  {
    case AUDIO_REQ_SET_CUR:

      Audio_VolumeCtl(*((int16_t *)(pbuf)));
      /* Add your code here */
      break;

    default:
      break;
  }
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
  //   BSP_AUDIO_IN_Init(BSP_AUDIO_IN_INSTANCE, &MicParams);
  return (USBD_OK);

}
extern uint8_t PDM_Buffer[];
static int8_t AUDIO_CDC_Record(void)
{
  //  BSP_AUDIO_IN_Record(BSP_AUDIO_IN_INSTANCE, (uint8_t *) PDM_Buffer, 0);
  return (USBD_OK);
}

/* Initialize software interrupt tasks */
/* Start audio acquisition */


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Add your own code here */
}

/**
  * @brief  Fills USB audio buffer with the right amount of data, depending on the
  *     channel/frequency configuration
  * @param  audioData: pointer to the PCM audio data
  * @param  PCMSamples: number of PCM samples to be passed to USB engine
  * @note Depending on the calling frequency, a coherent amount of samples must be passed to
  *       the function. E.g.: assuming a Sampling frequency of 16 KHz and 1 channel,
  *       you can pass 16 PCM samples if the function is called each millisecond,
  *       32 samples if called every 2 milliseconds and so on.
  */
void Send_Audio_to_USB(int16_t *audioData, uint16_t PCMSamples)
{

  USBD_AUDIO_Data_Transfer(&hUSBDDevice, (int16_t *)audioData, PCMSamples);
}

