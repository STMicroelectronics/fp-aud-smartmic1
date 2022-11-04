/**
******************************************************************************
* @file    STCmdP.h
* @author  Central Labs
* @version V0.0.1
* @date    11-Mar-2015
* @brief   This file contains definitions for STCmdP.h file.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __STCMDP__
#define __STCMDP__

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

#ifndef __weak
   #define __weak   __attribute__((weak))
 #endif /* __weak */

/** @addtogroup STCmdP
  * @{
  */

/** @addtogroup Command_Protocol
  * @{
  */ 

/* Exported constants --------------------------------------------------------*/

/** @defgroup Command_Protocol_Exported_Constants
  * @{
  */

/** @defgroup Command_Protocol_special_characters
  * @{
  */ 
#define TMsg_EOF                         0xF0   /*!< EOF of layer 1 packet    */
#define TMsg_BS                          0xF1   /*!< Byte stuffing escape     */
#define TMsg_BS_EOF                      0xF2   /*!< Substitution for TMsg_EOF*/
/**
  * @}
  */

/** @defgroup Command_Protocol_Msg_Length
  * @{
  */
#ifdef USE_USB_OTG_HS
  #define TMsg_MaxLen			 512    /*!< Message length           */
#else
  #define TMsg_MaxLen   		 512   /*!< Message length           */
#endif
/**
  * @}
  */

/**
  * @}
  */
    
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  Protocol message structure definition
  */   
typedef struct {
  uint32_t Len;                                 /*!< Message length           */
  uint8_t Data[TMsg_MaxLen];                    /*!< Message data             */
} TMsg;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/* Extract and Prepare message function ***************************************************/
uint16_t STCmdP_Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos, uint16_t BufMaxLen, TMsg *Msg);
uint16_t STCmdP_Prepare_Msg(uint8_t *Buffer, TMsg *Msg);
uint16_t STCmdP_Prepare_WaitingMsg(uint8_t *Buffer, TMsg *Msg, uint16_t timeout);
void STCmdP_WMsg_Received(TMsg *Msg);

/* Protocol Layer 1 functions *************************************************/
int STCmdP_ByteStuffCopyByte(uint8_t *Dest, uint8_t Source);
int STCmdP_ByteStuffCopy(uint8_t *Dest, TMsg *Source);
int STCmdP_ReverseByteStuffCopyByte2(uint8_t Source0, uint8_t Source1, uint8_t *Dest);
int STCmdP_ReverseByteStuffCopyByte(uint8_t *Source, uint8_t *Dest);
int STCmdP_ReverseByteStuffCopy(TMsg *Dest, uint8_t *Source);

/* Protocol Layer 2 functions *************************************************/
void STCmdP_CHK_ComputeAndAdd(TMsg *Msg);
int STCmdP_CHK_CheckAndRemove(TMsg *Msg);

/* Utility functions **********************************************************/
uint32_t STCmdP_Deserialize(uint8_t *Source, uint32_t Len);
int32_t STCmdP_Deserialize_s32(uint8_t *Source, uint32_t Len);
void STCmdP_Serialize(uint8_t *Dest, uint32_t Source, uint32_t Len);
void STCmdP_Serialize_s32(uint8_t *Dest, int32_t Source, uint32_t Len);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STCMDP__ */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
