/**
******************************************************************************
* @file    CommInterface_Template.c
* @author  Central Labs
* @version V0.0.1
* @date    11-Mar-2015
* @brief   This file contains an example that explain how to implement the 
*		   functions used for received and send message over the choosen interface
*          
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
/*Include --------------------------------------------------------------------*/
#include "CommInterface_Template.h"
#include "STCmdP.h"


/* Private variables ---------------------------------------------------------*/
uint16_t TEMPLATE_StartOfMsg_idx = 0;
uint16_t TEMPLATE_LastByteMsg_idx = 0;
uint16_t TEMPLATE_NewByte_idx = 0;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Send a Msg
  * @param  Msg pointer to the msg
  * @retval None
  */
void TEMPLATE_SendMsg(TMsg *Msg) 
{      
  uint8_t *TEMPLATE_TxBuffer;
  uint16_t CountOut;
  
  TEMPLATE_TxBuffer = TEMPLATE_GetTxBuffer();
  
  CountOut = STCmdP_Prepare_Msg(TEMPLATE_TxBuffer, Msg);

  send_via_interface(CountOut);
}


/**
  * @brief  Check if a message is received 
  * @param  Msg pointer to the msg 
  * @retval None
  */
int TEMPLATE_ReceivedMSG(TMsg *Msg)
{
  uint16_t numByteRead;
  uint8_t *TEMPLATE_RxBuffer;
 
  if (TEMPLATE_NewByte_idx != TEMPLATE_CheckForNewData())  
  {
    TEMPLATE_NewByte_idx = TEMPLATE_CheckForNewData();
    TEMPLATE_RxBuffer = TEMPLATE_GetRxBuffer();
        
    if(numByteRead = STCmdP_Extract_Msg(TEMPLATE_RxBuffer, TEMPLATE_StartOfMsg_idx, TEMPLATE_NewByte_idx, TEMPLATE_RxBufferDim, Msg)!=0)
    {
      TEMPLATE_StartOfMsg_idx = TEMPLATE_NewByte_idx;
      return 1;
    }
  }
  return 0;     
}

