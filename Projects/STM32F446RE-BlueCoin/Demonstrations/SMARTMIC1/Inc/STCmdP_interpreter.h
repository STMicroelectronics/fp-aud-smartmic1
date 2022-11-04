/**
******************************************************************************
* @file    STCmdP_interpreter.h
* @author  SRA
* 
* 
* @brief   This file contains definitions for STCmdP_interpreter.h file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STCMDP_INTERPRETER_H
#define __STCMDP_INTERPRETER_H

/* Includes ------------------------------------------------------------------*/

#include "audio_application.h"
#include "Audio_SerialCmd_Handlers.h"
#include "STCmdP_Command.h"

/** @defgroup SMARTMIC1 
* @{
*/

/** @defgroup SMARTMIC1_COMMUNICATION 
* @{
*/

/** @addtogroup SMARTMIC1_COMMUNICATION_STCmdPInterpreter
* @{
*/

/** @addtogroup SMARTMIC1_COMMUNICATION_STCmdPInterpreter_Exported_Define
* @{
*/
/* Exported define -----------------------------------------------------------*/
#define SENDER_UART     		0x01
#define SENDER_USB		        0x02
#define SENDER_SPI		        0x03
#define SENDER_BLE                      0x04
#define DEV_ADDR                        50
/**
* @}
*/ 

/** @addtogroup SMARTMIC1_COMMUNICATION_STCmdPInterpreter_Exported_Variables
* @{
*/
/* Exported variables --------------------------------------------------------*/
extern volatile uint8_t SenderInterface;
extern volatile uint8_t DataStreamingDest;
/**
* @}
*/ 

/** @addtogroup SMARTMIC1_COMMUNICATION_STCmdPInterpreter_Exported_Functions_Prototypes
* @{
*/
/* Exported functions prototypes ---------------------------------------------*/
int HandleMSG(TMsg *Msg);
void BUILD_REPLY_HEADER(TMsg *Msg);
void BUILD_NACK_HEADER(TMsg *Msg);
int Generic_ReceivedMSG(TMsg *Msg);
void Generic_SendMsg(TMsg *Msg);

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




#endif /* __STCMDP_INTERPRETER_H */


