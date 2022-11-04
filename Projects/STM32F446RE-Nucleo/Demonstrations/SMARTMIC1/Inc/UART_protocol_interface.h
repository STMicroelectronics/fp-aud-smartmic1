/**
******************************************************************************
* @file    UART_protocol_interface.h
* @author  SRA
* 
* 
* @brief   header for UART_protocol_interface.c.
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
#ifndef __UART_COMM__H
#define __UART_COMM__H

/* Includes ------------------------------------------------------------------*/
#include "STCmdP.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/** @defgroup SMARTMIC1 
* @{
*/

/** @defgroup SMARTMIC1_COMMUNICATION 
* @{
*/

/** @defgroup SMARTMIC1_COMMUNICATION_UART_Protocol_Interface
* @{
*/

/** @defgroup SMARTMIC1_COMMUNICATION_UART_Protocol_Interface_Exported_Function_Prototypes
* @{
*/

int UART_ReceivedMSG(TMsg *Msg);
void UART_SendMsg(TMsg *Msg);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __UART_COMM__H */
