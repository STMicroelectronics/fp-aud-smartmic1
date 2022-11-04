/**
******************************************************************************
* @file    sta350bw.c
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
#include "sta350bw.h"

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
/* Audio codec driver structure initialization */
STA350BW_AUDIO_Drv_t STA350BW_AUDIO_Driver = 
{ 
  STA350BW_Init,
  STA350BW_DeInit,
  STA350BW_ReadID,
  STA350BW_Play,
  STA350BW_Pause,
  STA350BW_Resume,
  STA350BW_Stop,
  STA350BW_SetFrequency,
  STA350BW_GetFrequency,
  STA350BW_SetVolume,
  STA350BW_GetVolume,
  STA350BW_SetMute,  
  STA350BW_SetOutputMode,
  STA350BW_SetResolution, 
  STA350BW_GetResolution, 
  STA350BW_SetProtocol,   
  STA350BW_GetProtocol,   
  STA350BW_Reset         
};
  
/**
* @}
*/

/** @defgroup STA350BW_Private_FunctionPrototypes
* @{
*/
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);

/**
* @}
*/

/** @defgroup STA350BW_Private_Functions
* @{
*/

int32_t STA350BW_RegisterBusIO(STA350BW_Object_t *pObj, STA350BW_IO_t *pIO)
{
  int32_t ret;
  
  if (pObj == NULL)
  {
    ret = STA350BW_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;
    
    pObj->Ctx.read_reg  = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;
    
    if (pObj->IO.Init != NULL)
    {
      ret = pObj->IO.Init();
    }
    else
    {
      ret = STA350BW_ERROR;
    }
  }
  
  return ret;
}

/**
* @brief        Initializes the STA350BW and the control interface.
* @param        handle: object related to the current device instance.
* @param        volume: master volume to be setup.
* @param        samplingFreq: sampling frequency.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Init(STA350BW_Object_t *pObj, void *params)
{
  if (pObj->is_initialized == 0U)
  {
    if (sta350bw_Init(&pObj->Ctx, 0x22, (uint32_t) 32000) != STA350BW_OK)
    {
      return STA350BW_ERROR;
    }
  }
  
  pObj->is_initialized = 1;
  
  return STA350BW_OK;	
}

/**
* @brief        Deinitializes the STA350BW and the control interface.
* @param        handle: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_DeInit(STA350BW_Object_t *pObj) 
{  
  if (pObj->is_initialized == 1U)
  {
    sta350bw_DeInit(&(pObj->Ctx));
    pObj->is_initialized = 0;
  }
  
  return STA350BW_OK;  
}

/**
* @brief        Read the device ID.
* @param        handle: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_ReadID(STA350BW_Object_t *pObj, uint32_t *Id)
{
  if (sta350bw_ReadID(&(pObj->Ctx)) != STA350BW_OK)
  {
    return STA350BW_ERROR;
  }
  
  return STA350BW_OK;
}

/**
* @brief        Start the audio play.
* @param        handle: object related to the current device instance.
* @param        *pData: pointer to audio data.
* @param        *p: pointer to optional additional functions.
* @param        Size: size of the data buffer.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Play(STA350BW_Object_t *pObj, uint16_t* pBuffer, uint16_t Size)
{
  if (pObj->audio_is_enabled != 0)
  {
    return STA350BW_ERROR;
  }
  else if (sta350bw_Play(&(pObj->Ctx), pBuffer, Size) != STA350BW_OK)
  {
    return STA350BW_ERROR;
  }
  else
  {
    pObj->audio_is_enabled = 1;
  }
  
  return STA350BW_OK;
}

/**
* @brief        Pause the audio play.
* @param        handle: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Pause(STA350BW_Object_t *pObj) 
{
  if (pObj->audio_is_enabled == 1)
  {
    sta350bw_Pause(&(pObj->Ctx));
    pObj->audio_is_enabled = 0;
  }
  
  return STA350BW_OK;
}

/**
* @brief        Resume the audio play.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Resume(STA350BW_Object_t *pObj) 
{
  if (pObj->audio_is_enabled != 1)
  {
    return STA350BW_ERROR;
  }
  else if (sta350bw_Resume(&(pObj->Ctx)) != STA350BW_OK)
  {
    return STA350BW_ERROR;
  }
  else
  {
    pObj->audio_is_enabled = 1;
  }
  
  return STA350BW_OK;
}

/**
* @brief        Stop audio stream.
* @param        handle: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Stop(STA350BW_Object_t *pObj,uint32_t Cmd)
{    
  if (pObj->audio_is_enabled != 0)
  {
    return STA350BW_ERROR;
  }
  else if (sta350bw_Stop(&(pObj->Ctx)) != STA350BW_OK)
  {
    return STA350BW_ERROR;
  }
  else
  {
    pObj->audio_is_enabled = 0;
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
int32_t STA350BW_SetVolume(STA350BW_Object_t *pObj, uint32_t Cmd, uint8_t Volume)
{
  if (sta350bw_SetVolume(&(pObj->Ctx), Cmd, Volume) != STA350BW_OK)
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
int32_t STA350BW_SetMute(STA350BW_Object_t *pObj, uint32_t state) 
{  
  if (sta350bw_SetMute(&(pObj->Ctx), STA350BW_CHANNEL_MASTER, state) != STA350BW_OK)
  {
    return STA350BW_ERROR;
  }
  
  return STA350BW_OK;
}



int32_t STA350BW_SetOutputMode(STA350BW_Object_t *pObj, uint8_t Output)
{
  return STA350BW_OK;
}

/**
* @brief        set the sampling frequency for STA350BW.
* @param        handle: object related to the current device instance.
* @param        AudioFreq: audio frequency to be set
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_SetFrequency(STA350BW_Object_t *pObj, uint32_t AudioFreq)
{
  if (sta350bw_SetFrequency(&(pObj->Ctx), AudioFreq) != STA350BW_OK)	
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
int32_t STA350BW_Reset(STA350BW_Object_t *pObj)
{ 
  if ( sta350bw_Reset(&(pObj->Ctx)) != STA350BW_OK)	
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;  
}

int32_t STA350BW_SetResolution(void* a, uint32_t b)
{
  return STA350BW_NOT_IMPLEMENTED;
}

int32_t STA350BW_GetResolution(void* a, uint32_t* b)
{
  return STA350BW_NOT_IMPLEMENTED;
}

int32_t STA350BW_SetProtocol(void* a, uint32_t b)
{
  return STA350BW_NOT_IMPLEMENTED;
}

int32_t STA350BW_GetProtocol(void* a, uint32_t* b)
{
  return STA350BW_NOT_IMPLEMENTED;
}

int32_t STA350BW_GetVolume(void* a, uint32_t b, uint8_t* c)
{
  return STA350BW_NOT_IMPLEMENTED;
}

int32_t STA350BW_GetFrequency(void* a)
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
int32_t STA350BW_SetEq(STA350BW_Object_t *pObj, uint8_t ramBlock,
                       uint8_t filterNumber, uint32_t * filterValues) 
{  
  if (sta350bw_SetEq(&(pObj->Ctx), ramBlock, filterNumber, filterValues) != STA350BW_OK)	
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK; 
}

/**
* @brief        Set tone value in the STA350BW tone register.
* @param        handle: object related to the current device instance.
* @param        toneGain: gain of the tone control
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_SetTone(STA350BW_Object_t *pObj, uint8_t toneGain) 
{
  if (sta350bw_SetTone(&(pObj->Ctx),toneGain) != 0) 
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
int32_t  STA350BW_SetDSPOption(STA350BW_Object_t *pObj, uint8_t option, uint8_t state)
{
  if (sta350bw_SetDSPOption(&(pObj->Ctx),option,state) != 0) 
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}


/**
* @brief  Wrap Read register component function to Bus IO function
* @param  Handle the device handler
* @param  Reg the register address
* @param  pData the stored data pointer
* @param  Length the length
* @retval 0 in case of success, an error code otherwise
*/
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  STA350BW_Object_t *pObj = (STA350BW_Object_t *)Handle;
  
  if (pObj->IO.BusType == (uint32_t)STA350BW_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte read */
    return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
  }
  else /* SPI 3-Wires */
  {
    /* Enable Multi-byte read */
    return STA350BW_ERROR;
  }
}

/**
* @brief  Wrap Write register component function to Bus IO function
* @param  Handle the device handler
* @param  Reg the register address
* @param  pData the stored data pointer
* @param  Length the length
* @retval 0 in case of success, an error code otherwise
*/
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  STA350BW_Object_t *pObj = (STA350BW_Object_t *)Handle;
  
  if (pObj->IO.BusType == (uint32_t)STA350BW_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte write */
    return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
  }
  else /* SPI 3-Wires */
  {
    /* Enable Multi-byte write */
    return STA350BW_ERROR;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


