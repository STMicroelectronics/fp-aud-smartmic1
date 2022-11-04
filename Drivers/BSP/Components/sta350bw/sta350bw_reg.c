/**
******************************************************************************
* @file    sta350bw_reg.c
* @author  SRA - Central Labs
* @version v3.0.0
* @date    6-May-19
* @brief   This file provides the STA350BW SOUND TERMINAL audio driver.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "sta350bw_reg.h"

/** @addtogroup BSP
* @{
*/

/** @addtogroup Components
* @{
*/

/** @addtogroup STA350BW
* @{
*/

/** @defgroup STA350BW_Private_Types
* @{
*/

/**
* @}
*/

/** @defgroup STA350BW_Private_Defines
* @{
*/

/**
* @}
*/

/** @defgroup STA350BW_Private_Macros
* @{
*/

/**
* @}
*/

/** @defgroup STA350BW_Private_Variables
* @{
*/

/**
* @}
*/

/** @defgroup STA350BW_Private_FunctionPrototypes
* @{
*/

static int32_t writeRAMSet(STA350BW_ctx_t * handle, uint8_t RAM_block,
                           uint8_t RAM_address, uint8_t * pIn);

int32_t STA350BW_ReadReg(STA350BW_ctx_t *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
int32_t STA350BW_WriteReg(STA350BW_ctx_t *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );

/**
* @}
*/

/** @defgroup STA350BW_Private_Functions
* @{
*/

/**
* @brief        Initializes the STA350BW and the control interface.
* @param        handle: object related to the current device instance.
* @param        volume: master volume to be setup.
* @param        samplingFreq: sampling frequency.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_Init(STA350BW_ctx_t * handle, uint16_t volume,
                      uint32_t samplingFreq) 
{
  uint8_t tmp = 0x00;  
  
  /* Set Master clock depending on sampling frequency*/
  if (sta350bw_SetFrequency(handle, samplingFreq) != 0) 
  {
    return STA350BW_ERROR;
  }
  
  /*Read Status Register*/
  if (STA350BW_ReadReg(handle, STA350BW_STATUS, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;                          
  }
  
  if (tmp != 0x7F) 
  {
    /*Status register highlights undesired behaviour
    (PLL not locked, ...)*/
    return STA350BW_ERROR;
  }
  
  /*Setup Master volume */
  if (sta350bw_SetVolume(handle, STA350BW_CHANNEL_MASTER, volume) != 0) 
  {
    return STA350BW_ERROR;
  }
  
  if (STA350BW_ReadReg(handle, STA350BW_CONF_REGF, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  
  tmp &= ~0x80;
  tmp |= 0x80;
  
  /*Enable Power Out Stage*/
  if (STA350BW_WriteReg(handle, STA350BW_CONF_REGF, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }  
  return STA350BW_OK;
}

/**
* @brief        Deinitializes the STA350BW and the control interface.
* @param        handle: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_DeInit(STA350BW_ctx_t * handle) 
{  
  uint8_t tmp;  
  if (STA350BW_ReadReg(handle, STA350BW_CONF_REGF, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }  
  tmp &= ~0xC0;
  if (STA350BW_WriteReg(handle, STA350BW_CONF_REGF, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }  
  return STA350BW_OK; 
}

/**
* @brief        Read the device ID.
* @param        handle: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_ReadID(STA350BW_ctx_t * handle) 
{
  return STA350BW_NOT_IMPLEMENTED;
}

/**
* @brief        Start the audio play.
* @param        handle: object related to the current device instance.
* @param        *pData: pointer to audio data.
* @param        *p: pointer to optional additional functions.
* @param        Size: size of the data buffer.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_Play(STA350BW_ctx_t * handle, uint16_t* pBuffer, uint16_t Size)
{
  return STA350BW_NOT_IMPLEMENTED;
}

/**
* @brief        Pause the audio play.
* @param        handle: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_Pause(STA350BW_ctx_t * handle) 
{
  /* Mute the output*/
  if (sta350bw_SetMute(handle, STA350BW_CHANNEL_MASTER, STA350BW_ENABLE)
      != 0) 
  {
    return STA350BW_ERROR;
  }  
  return STA350BW_OK;
}

/**
* @brief        Resume the audio play.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_Resume(STA350BW_ctx_t * handle) 
{
  /* Unmute the output*/
  if (sta350bw_SetMute(handle, STA350BW_CHANNEL_MASTER, STA350BW_DISABLE)!= 0) 
  {
    return STA350BW_ERROR;
  }  
  return STA350BW_OK;
}

/**
* @brief        Control the mute features of the STA350BW.
* @param        handle: object related to the current device instance.
* @param        channel: channel to be muted.
*               This parameter can be a value of @ref STA350BW_channel_define
* @param        state: eable disable parameter
*               This parameter can be a value of @ref STA350BW_state_define
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_SetMute(STA350BW_ctx_t * handle, uint8_t channel, uint8_t state) 
{  
  uint8_t tmp;
  
  if (STA350BW_ReadReg(handle, STA350BW_MUTE, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }  
  if (state == STA350BW_ENABLE) 
  {
    tmp |= channel;
  } else 
  {
    tmp &= ~channel;
  }
  
  if (STA350BW_WriteReg(handle, STA350BW_MUTE, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Control the volume features of the STA350BW.
* @param        handle: object related to the current device instance.
* @param        channel: channel to be controlled.
*               This parameter can be a value of @ref STA350BW_channel_define
* @param        volume: volume to be set
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_SetVolume(STA350BW_ctx_t * handle, uint32_t channel,
                           uint8_t Volume) 
{
  /*Setup volume */
  uint8_t tmp = Volume;
  if (STA350BW_WriteReg(handle, STA350BW_MVOL + channel, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        set the sampling frequency for STA350BW.
* @param        handle: object related to the current device instance.
* @param        AudioFreq: audio frequency to be set
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_SetFrequency(STA350BW_ctx_t * handle, uint32_t AudioFreq) 
{
  uint8_t tmp;
  
  if (STA350BW_ReadReg(handle, STA350BW_CONF_REGA, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  
  tmp &= ~0x1F;
  
  if (AudioFreq == STA350BW_Fs_32000 || AudioFreq == STA350BW_Fs_44100
      || AudioFreq == STA350BW_Fs_48000) 
  {
    tmp |= STA350BW_MCLK_256_LR_48K;
  } 
  else if (AudioFreq == STA350BW_Fs_88200 || AudioFreq == STA350BW_Fs_96000) 
  {
    tmp |= STA350BW_MCLK_256_LR_96K;
  } else {
    return STA350BW_ERROR;
  }
  
  if (STA350BW_WriteReg(handle, STA350BW_CONF_REGA, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Stop audio stream.
* @param        handle: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_Stop(STA350BW_ctx_t * handle)
{  
  return STA350BW_NOT_IMPLEMENTED; 
}

/**
* @brief        Set equalization parameters for STA350BW biquad section.
* @param        handle: object related to the current device instance.
* @param        ramBlock: ram block to be set
* @param        filterNumber: filter number
* @param        *filterValues: pointer to a uint32_t array containing filter coefficients
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_SetEq(STA350BW_ctx_t * handle, uint8_t ramBlock,
                       uint8_t filterNumber, uint32_t * filterValues) 
{  
  /*5 is due to the ram adressing: first filter is on the adresses 0x00 to 0x04; the second is on 0x05 to 0x09 ...*/
  if (writeRAMSet(handle, ramBlock, filterNumber * 5, (uint8_t *) filterValues)!=0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Reset device.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_Reset(STA350BW_ctx_t * handle)
{  
  return STA350BW_NOT_IMPLEMENTED; 
}


/**
* @brief        Set tone value in the STA350BW tone register.
* @param        handle: object related to the current device instance.
* @param        toneGain: gain of the tone control
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t sta350bw_SetTone(STA350BW_ctx_t * handle, uint8_t toneGain) 
{
  uint8_t tmp = toneGain;
  
  if (STA350BW_WriteReg(handle, STA350BW_TONE, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}


/**
* @brief  This function can be used to set advanced DSP options in order to 
*         use advanced features on the STA350BW device.
* @param  handle: object related to the current device instance.
* @param  option: specific option to be setted up
*         This parameter can be a value of @ref STA350BW_DSP_option_selection 
* @param  state: state of the option to be controlled. Depending on the selected 
*         DSP feature to be controlled, this value can be either ENABLE/DISABLE 
*         or a specific numerical parameter related to the specific DSP function. 
*         This parameter can be a value of @ref STA350BW_state_define   
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t  sta350bw_SetDSPOption(STA350BW_ctx_t * handle, uint8_t option, uint8_t state)
{  
  uint8_t tmp = 0;  
  
  switch (option) 
  {
  case STA350BW_DSPB:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CONF_REGD, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x04;
      tmp |= state << 0x02;
      
      if (STA350BW_WriteReg(handle, STA350BW_CONF_REGD, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_HPB:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CONF_REGD, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x01;
      tmp |= state << 0x00;
      
      if (STA350BW_WriteReg(handle, STA350BW_CONF_REGD, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_DEMP:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CONF_REGD, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x02;
      tmp |= state << 0x01;
      
      if (STA350BW_WriteReg(handle, STA350BW_CONF_REGD, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_BQL:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CONF_REGD, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x08;
      tmp |= state << 0x04;
      
      if (STA350BW_WriteReg(handle, STA350BW_CONF_REGD, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_BQ5:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CONFX, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x04;
      tmp |= state << 0x02;
      
      if (STA350BW_WriteReg(handle, STA350BW_CONFX, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_BQ6:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CONFX, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x02;
      tmp |= state << 0x01;
      
      if (STA350BW_WriteReg(handle, STA350BW_CONFX, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_BQ7:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CONFX, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x01;
      tmp |= state << 0x00;
      
      if (STA350BW_WriteReg(handle, STA350BW_CONFX, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }    
  case STA350BW_C1EQBP:
    {
      if (STA350BW_ReadReg(handle, STA350BW_C1CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x02;
      tmp |= state << 0x01;
      
      if (STA350BW_WriteReg(handle, STA350BW_C1CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }    
  case STA350BW_C2EQBP:
    {
      if (STA350BW_ReadReg(handle, STA350BW_C2CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x02;
      tmp |= state << 0x01;
      
      if (STA350BW_WriteReg(handle, STA350BW_C2CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }    
  case STA350BW_C1TCB:
    {
      if (STA350BW_ReadReg(handle, STA350BW_C1CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x01;
      tmp |= state << 0x00;
      
      if (STA350BW_WriteReg(handle, STA350BW_C1CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }    
  case STA350BW_C2TCB:
    {
      if (STA350BW_ReadReg(handle, STA350BW_C2CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x01;
      tmp |= state << 0x00;
      
      if (STA350BW_WriteReg(handle, STA350BW_C2CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }    
  case STA350BW_C1VBP:
    {
      if (STA350BW_ReadReg(handle, STA350BW_C1CFG, 1,  &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x04;
      tmp |= state << 0x02;
      
      if (STA350BW_WriteReg(handle, STA350BW_C1CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }    
  case STA350BW_C2VBP:
    {
      if (STA350BW_ReadReg(handle, STA350BW_C2CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x04;
      tmp |= state << 0x02;
      
      if (STA350BW_WriteReg(handle, STA350BW_C2CFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_EXT_RANGE_BQ1:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CXT_B4B1, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x03;
      tmp |= (state>>1);
      
      if (STA350BW_WriteReg(handle, STA350BW_CXT_B4B1, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_EXT_RANGE_BQ2:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CXT_B4B1, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x0C;
      tmp |= (state>>1) << 2;
      
      if (STA350BW_WriteReg(handle, STA350BW_CXT_B4B1, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_EXT_RANGE_BQ3:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CXT_B4B1, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x30;
      tmp |= (state>>1) << 4;
      
      if (STA350BW_WriteReg(handle, STA350BW_CXT_B4B1, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_EXT_RANGE_BQ4:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CXT_B4B1, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0xC0;
      tmp |= (state>>1) << 6;
      
      if (STA350BW_WriteReg(handle, STA350BW_CXT_B4B1, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_EXT_RANGE_BQ5:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CXT_B7B5, 1,&tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x03;
      tmp |= (state>>1);
      
      if (STA350BW_WriteReg(handle, STA350BW_CXT_B7B5, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_EXT_RANGE_BQ6:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CXT_B7B5, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x0C;
      tmp |= (state>>1) << 2;
      
      if (STA350BW_WriteReg(handle, STA350BW_CXT_B7B5, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_EXT_RANGE_BQ7:
    {
      if (STA350BW_ReadReg(handle, STA350BW_CXT_B7B5, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x30;
      tmp |= (state>>1) << 4;
      
      if (STA350BW_WriteReg(handle, STA350BW_CXT_B7B5, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    }
  case STA350BW_RAM_BANK_SELECT:
    {
      if (STA350BW_ReadReg(handle, STA350BW_EQCFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }  
      tmp &= ~0x03;
      tmp |= state;
      
      if (STA350BW_WriteReg(handle, STA350BW_EQCFG, 1, &tmp) != 0) 
      {
        return STA350BW_ERROR;
      }
      break;
    } 
  }
  return STA350BW_OK;
}

/**
* @brief        private function for writing a RAM set.
* @param        handle: object related to the current device instance.
* @param        RAM_block: ram block to be written.
* @param        RAM_address: ram address to be written.
* @param        *pIn: pointer to the desired value to be write.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
static int32_t writeRAMSet(STA350BW_ctx_t * handle, uint8_t RAM_block,
                           uint8_t RAM_address, uint8_t * pIn) 
{
  uint8_t tmp = 0x00;
  /*choose block*/
  if (STA350BW_ReadReg(handle, STA350BW_EQCFG, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  tmp &= ~0x03;
  RAM_block &= 0x03;
  tmp |= RAM_block;
  if (STA350BW_WriteReg(handle, STA350BW_EQCFG, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  /*set address*/
  if (STA350BW_ReadReg(handle, STA350BW_CFADDR, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  tmp &= ~0x3F;
  RAM_address &= 0x3F;
  tmp |= RAM_address;
  if (STA350BW_WriteReg(handle, STA350BW_CFADDR, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  /*write*/
  if (STA350BW_WriteReg(handle, STA350BW_B1CF1, 1, &pIn[2]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_B1CF2, 1, &pIn[1]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_B1CF3, 1, &pIn[0]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_B2CF1, 1, &pIn[6]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_B2CF2, 1, &pIn[5]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_B2CF3, 1, &pIn[4]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_A1CF1, 1, &pIn[10]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_A1CF2, 1, &pIn[9]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_A1CF3, 1, &pIn[8]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_A2CF1, 1, &pIn[14]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_A2CF2, 1, &pIn[13]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_A2CF3, 1, &pIn[12]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_B0CF1, 1, &pIn[18]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_B0CF2, 1, &pIn[17]) != 0) 
  {
    return STA350BW_ERROR;
  }
  if (STA350BW_WriteReg(handle, STA350BW_B0CF3, 1, &pIn[16]) != 0) 
  {
    return STA350BW_ERROR;
  }  
  /*Set WA PIN*/
  if (STA350BW_ReadReg(handle, STA350BW_CFUD, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  tmp &= ~0x02;
  tmp = 0x02; 
  
  if (STA350BW_WriteReg(handle, STA350BW_CFUD, 1, &tmp) != 0) 
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Generic reading function. It must be fullfilled with either I2C 
*               or SPI writing function.
* @param        RegAddr address of the register.
* @param        NumByteToRead Size to be read.
* @param        *Data buffer to be filled with read data.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_ReadReg(STA350BW_ctx_t *ctx, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, RegAddr, Data, NumByteToRead);
  return ret;
}

/**
* @brief        Generic writing function. It must be fullfilled with either I2C 
*               or SPI writing function.
* @param        RegAddr address of the register.
* @param        NumByteToWrite Size to be written.
* @param        *Data buffer to be written.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_WriteReg(STA350BW_ctx_t *ctx, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data )
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, RegAddr, Data, NumByteToWrite);
  return ret;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


