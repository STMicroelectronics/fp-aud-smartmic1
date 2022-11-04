/**
******************************************************************************
* @file    usb_protocol_interface.h
* @author  SRA
* 
* 
* @brief   header for usb_protocol_interface.c.
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


/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __USB_PROTOCOL_INTERFACE__H
#define __USB_PROTOCOL_INTERFACE__H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "STCmdP.h"

/** @addtogroup STEVAL-IDI001V1_Applications
* @{
*/

/** @addtogroup Data_Logger
* @{
*/

void USB_SendMsg(TMsg *Msg);
int USB_ReceivedMSG(TMsg *Msg);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_PROTOCOL_INTERFACE__H */



