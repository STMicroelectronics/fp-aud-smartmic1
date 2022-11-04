/**
  ******************************************************************************
  * @file    UART_protocol_interface.c
  * @author  SRA
  * 
  * 
  * @brief   This file contains the functions used for received and send message
  *          via UART
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo_USART.h"
#include "UART_protocol_interface.h"

/* Private variables ---------------------------------------------------------*/


/** @defgroup SMARTMIC1
  * @{
  */

/** @defgroup SMARTMIC1_COMMUNICATION
  * @{
  */

/** @defgroup SMARTMIC1_COMMUNICATION_UART_Protocol_Interface
  * @{
  */

/** @defgroup SMARTMIC1_COMMUNICATION_UART_Protocol_Interface_Private_Variables
  * @{
  */

uint16_t UART_StartOfMsg_idx = 0;
uint16_t UART_LastByteMsg_idx = 0;
uint16_t UART_NewByte_idx = 0;

/**
  * @}
  */

/** @defgroup SMARTMIC1_COMMUNICATION_UART_Protocol_Interface_Exported_Functions
  * @{
  */

/**
  * @brief  Check if a message is received via UART
  * @param  Msg pointer to the message
  * @retval Error (0,1) 1 if a complete message is found
  */
int UART_ReceivedMSG(TMsg *Msg)
{
  uint8_t *UART_RxBuffer;

  if (UART_NewByte_idx != BSP_USART_CheckForNewData())
  {
    UART_NewByte_idx = BSP_USART_CheckForNewData();
    UART_RxBuffer = BSP_USART_GetRxBuffer();

    if ((STCmdP_Extract_Msg(UART_RxBuffer, UART_StartOfMsg_idx, UART_NewByte_idx, BSP_USART_RX_BUFFER_SIZE, Msg)) != 0)
    {
      UART_StartOfMsg_idx = UART_NewByte_idx;
      return 1;
    }
  }
  return 0;
}


/**
  * @brief  Send message via UART
  * @param  Msg pointer to the message
  * @retval None
  */
void UART_SendMsg(TMsg *Msg)
{
  uint8_t *UART_TxBuffer;
  uint16_t CountOut;

  UART_TxBuffer = BSP_USART_GetTxBuffer();

  CountOut = STCmdP_Prepare_Msg(UART_TxBuffer, Msg);

  BSP_USART_SendData(CountOut);
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

