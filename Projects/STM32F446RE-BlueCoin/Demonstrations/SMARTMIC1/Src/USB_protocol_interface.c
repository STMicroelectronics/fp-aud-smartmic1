/**
  ******************************************************************************
  * @file    usb_protocol_interface.c
  * @author  SRA
  * 
  * 
  * @brief   This file contains the functions used for received and send message
  *          via USB
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

/*Include --------------------------------------------------------------------*/
#include "usb_protocol_interface.h"
#include "STCmdP.h"
#include "usbd_audio_cdc_interface.h"


/** @addtogroup STEVAL-IDI001V1_Applications
  * @{
  */

/** @addtogroup Data_Logger
  * @{
  */

/** @defgroup USB_PROTOCOL_INTERFACE
  * @{
  */

/** @defgroup USB_PROTOCOL_INTERFACE_Private_Variables
  * @{
  */
/* Private variables ---------------------------------------------------------*/
uint16_t USB_StartOfMsg_idx = 0;
uint16_t USB_LastByteMsg_idx = 0;
uint16_t USB_NewByte_idx = 0;

uint8_t  My_Buffer[2 * TMsg_MaxLen];
/**
  * @}
  */

/** @defgroup USB_PROTOCOL_INTERFACE_Exported_Functions
  * @{
  */
/**
  * @brief  Send a Msg via USB
  * @param  Msg pointer to the msg
  * @retval None
  */
void USB_SendMsg(TMsg *Msg)
{
  uint16_t CountOut;

  CountOut = STCmdP_Prepare_Msg((uint8_t *)My_Buffer, Msg);

  CDC_Fill_Buffer((uint8_t *)My_Buffer, CountOut);
}

/**
  * @brief  Check if a message is received via USB.
  * @param  Msg pointer to the msg
  * @retval None
  */
int USB_ReceivedMSG(TMsg *Msg)
{
  uint8_t *USB_RxBuffer;

  if (USB_NewByte_idx != USB_CheckForNewData())
  {
    USB_NewByte_idx = USB_CheckForNewData();
    USB_RxBuffer = USB_GetRxBuffer();

    if (STCmdP_Extract_Msg(USB_RxBuffer, USB_StartOfMsg_idx, USB_NewByte_idx, USB_RxBufferDim, Msg) != 0)
    {
      USB_StartOfMsg_idx = USB_NewByte_idx;
      return 1;
    }
  }
  return 0;
}


/**
  * @brief  Check if a message is received via USB.
  * @param  Msg pointer to the msg
  * @retval None
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


