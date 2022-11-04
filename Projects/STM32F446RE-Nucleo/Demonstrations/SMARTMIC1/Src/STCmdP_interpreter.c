/**
  ******************************************************************************
  * @file    STCmpP_interpreter.c
  * @author  SRA
  * 
  * 
  * @brief   Handler STCmdP Protocol
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
#include "STCmdP_interpreter.h"

/** @defgroup SMARTMIC1
  * @{
  */

/** @defgroup SMARTMIC1_COMMUNICATION
  * @{
  */

/** @addtogroup SMARTMIC1_COMMUNICATION_STCmdPInterpreter
  * @{
  */

/** @addtogroup SMARTMIC1_COMMUNICATION_STCmdPInterpreter_Private_Variables
  * @{
  */
/* Private variabels ---------------------------------------------------------*/
#ifdef COUPONS
uint8_t PresentationString[] = {"OSX SMARTMIC1 | NucleoSystem STM32F446 |Microphone Coupons|1.0.0"};
#elif defined (u4_ARRAY)
uint8_t PresentationString[] = {"OSX SMARTMIC1 | NucleoSystem STM32F446 |u4 Array| 1.0.0"};
#endif

volatile uint8_t DataStreamingDest = 0x01;
/**
  * @}
  */

/** @addtogroup SMARTMIC1_COMMUNICATION_STCmdPInterpreter_Exported_Variables
  * @{
  */
/* Exported variabels --------------------------------------------------------*/
volatile uint8_t SenderInterface = 0;
/**
  * @}
  */

/** @addtogroup SMARTMIC1_COMMUNICATION_STCmdPInterpreter_Exported_Functions
  * @{
  */
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Build reply msg header
  * @param  Msg pointer to the reply msg
  * @retval None
  */
void BUILD_REPLY_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_Reply_Add;
  Msg->Len = 3;
}

/**
  * @brief  Build reply failed msg header
  * @param  Msg pointer to the reply msg
  * @retval None
  */
void BUILD_NACK_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_NACK;
}

/**
  * @brief  Send masg through selected interface
  * @param  Msg pointer to the sending msg
  * @retval None
  */
void Generic_SendMsg(TMsg *Msg)
{
#ifndef USE_COMPOSITE_VCP_AUDIO
  UART_SendMsg(Msg);
#else
  USB_SendMsg(Msg);
#endif
}

/**
  * @brief  Check if new msg is been received on the selected interface
  * @param  Msg pointer for the new msg
  * @retval None
  */
int Generic_ReceivedMSG(TMsg *Msg)
{
#ifndef USE_COMPOSITE_VCP_AUDIO
  if (UART_ReceivedMSG(Msg))
#else
  if (USB_ReceivedMSG(Msg))
#endif
  {
    SenderInterface = SENDER_UART;
    return 1;
  }
  return 0;
}

/**
  * @brief  Handle the received msg
  * @param  Msg pointer to the new msg
  * @retval None
  */
int HandleMSG(TMsg *Msg)
/* | DestAddr | SouceAddr | CMD | PAYLOAD |
|   1      |   1       |  1  |    N    |     */
{
  const uint8_t *p1;
  uint8_t *p2;
  uint32_t i;

  if (Msg->Len < 2) { return 0; }
  if (Msg->Data[0] != DEV_ADDR) { return 0; }
  switch (Msg->Data[2])   /** CMD **/
  {

    case CMD_Ping:
      if (Msg->Len != 3) { return 0; }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      return 1;

    case CMD_InterfaceType:
      if (Msg->Len != 3) { return 0; }
      BUILD_REPLY_HEADER(Msg);
      Msg->Data[3] = SenderInterface;
      Msg->Len = 4;
      return 1;

    case CMD_Enter_DFU_Mode:
      if (Msg->Len != 3) { return 0; }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      return 1;

    case CMD_Read_PresString:
      if (Msg->Len != 3) { return 0; }
      BUILD_REPLY_HEADER(Msg);
      p1 = PresentationString;
      p2 = &Msg->Data[3];
      i = 0;
      while (*p1)
      {
        *p2 = *p1;
        p1++;
        p2++;
        i++;
      }
      *p2 = 0;
      i++;
      Msg->Len = 3 + i;
      return 1;

    case CMD_AudioModule_GetStatus:
      if (Msg->Len < 3) { return 0; }
      BUILD_REPLY_HEADER(Msg);
      Handle_CMD_AUDIO_IN_GetStatus(Msg);
      return 1;

    case CMD_AudioModule_SetStatus:
      if (Msg->Len < 3) { return 0; }
      BUILD_REPLY_HEADER(Msg);
      Handle_CMD_AUDIO_IN_SetStatus(Msg);
      return 1;

    case CMD_Reset:
      return 1;

    default:
      return 0;
  }
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

