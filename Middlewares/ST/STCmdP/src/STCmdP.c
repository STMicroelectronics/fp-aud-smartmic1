/**
******************************************************************************
* @file    STCmdP.c
* @author  Central Labs
* @version V0.0.1
* @date    11-Mar-2015
* @brief   This file implements the ST command protocol functions
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

/* Include ----------------------------------------------------------*/
#include "STCmdP.h"

/** @addtogroup STCmdP
  * @{
  */

/** @defgroup Command_Protocol
  * @brief Command Protocol
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** 
  * @brief  Waiting msg typedef
  */   
typedef struct {
  uint16_t code;                                /*!< Message code             */
  uint16_t timeout;                             /*!< Message timeout          */
} WaitingMsg;


/* Private define ------------------------------------------------------------*/
/** 
  * @brief  Max number waiting Msg
  */  
#define MAX_NUMBER_WAITING_MSG                  100
  

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/** 
  * @brief  Waiting msg list
  */  
WaitingMsg WaitingMsgList[MAX_NUMBER_WAITING_MSG];

/** 
  * @brief  Waiting msg index list
  */ 
int16_t lastWMsg_idx = -1;


/* Private function prototypes -----------------------------------------------*/
/** 
  * @brief  Check if a waiting msg is been received
  */ 
uint16_t STCmdP_Extract_WaitingMsg(TMsg *Msg);


/* Private functions ---------------------------------------------------------*/

/** @defgroup Command_Protocol_Private_Functions
  * @{
  */

/** @defgroup Command_Protocol_Group1 Extract and Prepare message function
 *  @brief    Extract and Prepare message functions. 
 *
@verbatim    
 ===============================================================================
                  ##### Extract and Prepare message functions #####
 ===============================================================================
    [..]  This section provides functions allowing to extract a protocol 
          message from a circular buffer
      
@endverbatim
  * @{
  */

/**
* @brief  extract a message from a circular buffer
* @param  CircularBuff circular buffer with data
* @param  InitPos first byte of the message
* @param  LastPos last added byte position
* @param  BufMaxLen buffer length
* @param  Msg pointer to the protocol message
* @retval  number of bytes read if the msg is finished
*/
uint16_t STCmdP_Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos, uint16_t BufMaxLen, TMsg *Msg)
{
  /* Number of bytes to be analysed */
  uint16_t NumNewByte = 0;    
  /* Byte to be checked*/  
  uint8_t Data;
  /* Circular buffer index */
  uint16_t BuffCounter;
  /* Two index for ByteStuffing process  */
  uint16_t BSCounter, BSCounterNext;
  /* Final message counter */
  uint16_t MsgCounter = 0;
  /* Number of input bytes processed by the ByteStuffing process */
  uint8_t inc = 0;
  
  if (LastPos >= StartPos) 
  {
    NumNewByte = LastPos - StartPos;
  } 
  else 
  {
    NumNewByte = BufMaxLen + LastPos - StartPos;
  }
  BSCounter = StartPos;
    
  for (BuffCounter = 0; BuffCounter < NumNewByte; BuffCounter++)
  {
    Data = CircularBuff[BSCounter];
    BSCounter++;
    if (BSCounter >= BufMaxLen) 
    {
      BSCounter = 0;
    }
    
    /* If End of message is found, start to recompose the message */
    if (Data == TMsg_EOF) 
    {
      BSCounter = StartPos;

      /* Extract Layer2 packet from Layer1 packet */
      for (uint16_t i = 0; i < BuffCounter; i += inc)
      {
        BSCounterNext = (BSCounter + 1) % BufMaxLen;
        inc = STCmdP_ReverseByteStuffCopyByte2(CircularBuff[BSCounter], CircularBuff[BSCounterNext], &Msg->Data[MsgCounter]);
        if (inc == 0) 
        {
          StartPos = BSCounterNext;
          return 0;
        }
        BSCounter = (BSCounter + inc) % BufMaxLen;
        MsgCounter++;
      }
      Msg->Len = MsgCounter;
      BSCounter = (BSCounter + 1) % BufMaxLen;
      StartPos = BSCounter;
      
      /* Check message integrity; extract Msg from Layer2 packet */
      if (STCmdP_CHK_CheckAndRemove(Msg)) 
      {  
        if(STCmdP_Extract_WaitingMsg(Msg))
        {
          return 0;
        }
        else
        {
          return NumNewByte;
        }
      }
    }
  }
  return 0;
}


/**
* @brief  extract a message from a circular buffer and remove it from the waiting list
* @param  CircularBuff circular buffer with data
* @param  InitPos first byte of the message
* @param  LastPos last added byte position
* @param  BufMaxLen buffer length
* @param  Msg pointer to the protocol message
* @retval  number of bytes read if the msg is finished
*/
uint16_t STCmdP_Extract_WaitingMsg(TMsg *Msg)
{
  uint16_t listCounter; 

  for(listCounter = 0; listCounter<=lastWMsg_idx; listCounter++)
  {
    if(WaitingMsgList[listCounter].code == Msg->Data[2])
    {
      STCmdP_WMsg_Received(Msg);
      WaitingMsgList[listCounter] = WaitingMsgList[lastWMsg_idx];
      lastWMsg_idx--;    
      return 1;
    }
  }
  return 0;
}

/**
  * @brief  Handle Waiting Msg received
  * @param  Msg: pointer to the waiting Msg received
  * @retval None
  */
__weak void STCmdP_WMsg_Received(TMsg *Msg)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the PROTOCOL_WMsg_Received could be implemented in the user file
   */
}

/**
* @brief  Prepare Msg to be sent
* @param  Buffer contains the data to be sent
* @param  Msg pointer to the protocol message
* @retval  None
*/
uint16_t STCmdP_Prepare_Msg(uint8_t *Buffer, TMsg *Msg)
{
  STCmdP_CHK_ComputeAndAdd(Msg);

  return (uint16_t)STCmdP_ByteStuffCopy(Buffer, Msg);
}


/**
* @brief  Prepare waiting Msg to be sent
* @param  Buffer contains the data to be sent
* @param  Msg pointer to the protocol message
* @param  Timeout for the msg response
* @retval  None
*/
uint16_t STCmdP_Prepare_WaitingMsg(uint8_t *Buffer, TMsg *Msg, uint16_t timeout)
{
  WaitingMsg WMsg;
  WMsg.code = Msg->Data[2];
  WMsg.timeout = timeout;
  
  STCmdP_CHK_ComputeAndAdd(Msg);
  
  lastWMsg_idx++;
  WaitingMsgList[lastWMsg_idx] = WMsg;
  
  return (uint16_t)STCmdP_ByteStuffCopy(Buffer, Msg);
}

/**
  * @}
  */

/** @defgroup Command_Protocol_Group2 Protocol Layer 1 functions 
 *  @brief    Protocol Layer 1 functions. 
 *
@verbatim    
 ===============================================================================
                  ##### Protocol Layer 1 functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Byte stuffing: is a process that transforms a sequence of data bytes
          that may contain 'illegal' or 'reserved' values into a potentially 
          longer sequence that contains no occurrences of those values.
      (+) Reverse the byte stuffing process in order to obtain the Layer2 packet
 
@endverbatim
  * @{
  */

/**
* @brief  Byte stuffing process for one byte 
* @param  Dest destination
* @param  Source source
* @retval  None
*/
int STCmdP_ByteStuffCopyByte(uint8_t *Dest, uint8_t Source)
{
  switch(Source) 
  {
    case TMsg_EOF:
      Dest[0]=TMsg_BS;
      Dest[1]=TMsg_BS_EOF;
      return 2;
    case TMsg_BS:
      Dest[0]=TMsg_BS;
      Dest[1]=TMsg_BS;
      return 2;
    default:
      Dest[0]=Source;
      return 1;
  }
}

/**
* @brief  Byte stuffing process for a Msg
* @param  Dest destination
* @param  Source source
* @retval  None
*/
int STCmdP_ByteStuffCopy(uint8_t *Dest, TMsg *Source)
{
  int i, Count;
  
  Count=0;
  for (i=0; i<Source->Len; i++) 
  {
    Count+=STCmdP_ByteStuffCopyByte(&Dest[Count], Source->Data[i]);
  }
  Dest[Count]=TMsg_EOF;
  Count++;
  return Count;
}

/**
* @brief  Reverse Byte stuffing process for one byte 
* @param  Source source
* @param  Dest destination
* @retval  None
*/
int STCmdP_ReverseByteStuffCopyByte(uint8_t *Source, uint8_t *Dest)
{
  if (Source[0] == TMsg_BS) 
  {
    if (Source[1] == TMsg_BS) 
    {
      *Dest=TMsg_BS;
      return 2;
    }
    if (Source[1] == TMsg_BS_EOF) 
    {
      *Dest=TMsg_EOF;
      return 2;
    }
    return 0; // invalide sequence
  }
  else 
  {
    *Dest=Source[0];
    return 1;
  }  
}

/**
* @brief  Reverse Byte stuffing process for two input data
* @param  Source0 input data  
* @param  Source1 input data  
* @param  Dest the destination data
* @retval  number of input bytes processed (1 or 2) or 0 for invalid sequence
*/
int STCmdP_ReverseByteStuffCopyByte2(uint8_t Source0, uint8_t Source1, uint8_t *Dest)
{
  if (Source0 == TMsg_BS)
  {
    if (Source1 == TMsg_BS) 
    {
      *Dest=TMsg_BS;
      return 2;
    }
    if (Source1 == TMsg_BS_EOF) 
    {
      *Dest=TMsg_EOF;
      return 2;
    }
    return 0; // invalid sequence
  } 
  else 
  {
    *Dest=Source0;
    return 1;
  }
}

/**
* @brief  Reverse Byte stuffing process for a Msg
* @param  Dest destination
* @param  Source source
* @retval  None
*/
int STCmdP_ReverseByteStuffCopy(TMsg *Dest, uint8_t *Source)
{
  int Count=0, State=0;

  while ((*Source)!=TMsg_EOF) 
  {
    if (State==0) 
    {
      if ((*Source)==TMsg_BS) 
      {
        State=1;
      }
      else 
      {
        Dest->Data[Count]=*Source;
        Count++;
      }
    } 
    else 
    {
      if ((*Source)==TMsg_BS) 
      {
        Dest->Data[Count]=TMsg_BS;
        Count++;
      } 
      else 
      {
        if ((*Source)==TMsg_BS_EOF) 
        {
          Dest->Data[Count]=TMsg_EOF;
          Count++;
        } 
        else 
        {
          return 0; // invalid sequence
        }
      }
      State=0;
    }
    Source++;
  }
  if (State!=0) return 0;
  Dest->Len=Count;
  return 1;
}

/**
  * @}
  */

/** @defgroup Command_Protocol_Group3 Protocol Layer 2 functions 
 *  @brief    Protocol Layer 2 functions. 
 *
@verbatim    
 ===============================================================================
                  ##### Protocol Layer 2 functions #####
 ===============================================================================
    [..]  This section provides functions allowing to ensures that the packet 
          handled contains the correct information
 
@endverbatim
  * @{
  */

/**
* @brief  Compute and add checksum
* @param  Msg pointer to the message
* @retval  None
*/
void STCmdP_CHK_ComputeAndAdd(TMsg *Msg)
{
  uint8_t CHK=0;
  int i;
  
  for(i=0; i<Msg->Len; i++) 
  {
    CHK-=Msg->Data[i];
  }
  Msg->Data[i]=CHK;
  Msg->Len++;
}


/**
* @brief  Compute and remove checksum 
* @param  Msg pointer to the message
* @retval  None
*/
int STCmdP_CHK_CheckAndRemove(TMsg *Msg)
{
  uint8_t CHK=0;
  int i;
  
  for(i=0; i<Msg->Len; i++) 
  {
    CHK+=Msg->Data[i];
  }
  Msg->Len--;
  return (CHK==0);
}

/**
  * @}
  */

/** @defgroup Command_Protocol_Group4 Utility functions 
 *  @brief    Utility functions. 
 *
@verbatim    
 ===============================================================================
                          ##### Utility functions #####
 ===============================================================================
    [..]  This section provides functions allowing to build or unbuild array 
          from int variables

@endverbatim
  * @{
  */

/**
* @brief  Build an array from the uint32_t (LSB first)
* @param  Dest destination
* @param  Source source
* @param  Len number of bytes
* @retval  None
*/
void STCmdP_Serialize(uint8_t *Dest, uint32_t Source, uint32_t Len)
{
  int i;
  for (i=0; i<Len; i++) 
  {
    Dest[i] = Source & 0xFF;
    Source>>=8;
  } 
}

/**
* @brief  Unbuild a Number from an array (LSB first) 
* @param  Source source
* @param  Len number of bytes
* @retval  Rebuild unsigned int variable
*/
uint32_t STCmdP_Deserialize(uint8_t *Source, uint32_t Len)
{
  uint32_t app;
  app=Source[--Len];
  while(Len>0) 
  {
    app<<=8;
    app+=Source[--Len];
  }
  return app;
}

/**
* @brief  Build an array from the uint32_t (LSB first)
* @param  Dest destination
* @param  Source source
* @param  Len number of bytes
* @retval  None
*/
void STCmdP_Serialize_s32(uint8_t *Dest, int32_t Source, uint32_t Len)
{
  int i;
  for (i=0; i<Len; i++)
  {
    Dest[i] = Source & 0xFF;
    Source>>=8;
  } 
}

/**
* @brief  Unbuild a Number from an array (LSB first) 
* @param  Source source
* @param  Len number of bytes
* @retval  Rebuild signed int variable
*/
int32_t STCmdP_Deserialize_s32(uint8_t *Source, uint32_t Len) 
{
  int32_t app;
  app=Source[--Len];
  while(Len>0) 
  {
    app<<=8;
    app+=Source[--Len];
  }
  return app;
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
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
