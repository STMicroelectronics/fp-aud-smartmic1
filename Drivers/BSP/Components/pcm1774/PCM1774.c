/**
 ******************************************************************************
 * @file    pcm1774.c
 * @author  SRA - Central Labs
 * @brief   PCM1774 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "PCM1774.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @defgroup PCM1774 PCM1774
 * @{
 */

/** @defgroup PCM1774_Exported_Variables PCM1774 Exported Variables
 * @{
 */

PCM1774_AUDIO_Drv_t PCM1774_AUDIO_Driver =
{
  PCM1774_Init,
  PCM1774_DeInit,
  PCM1774_ReadID,
  PCM1774_Play,
  PCM1774_Pause,
  PCM1774_Resume,
  PCM1774_Stop,
  PCM1774_SetFrequency,
  PCM1774_GetFrequency,
  PCM1774_SetVolume,
  PCM1774_GetVolume,
  PCM1774_SetMute,  
  PCM1774_SetOutputMode,
  PCM1774_SetResolution, 
  PCM1774_GetResolution,   
  PCM1774_SetProtocol,   
  PCM1774_GetProtocol,   
  PCM1774_Reset         
};


/**
 * @}
 */

/** @defgroup PCM1774_Private_Function_Prototypes PCM1774 Private Function Prototypes
 * @{
 */

static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);

/**
 * @}
 */

/** @defgroup PCM1774_Exported_Functions PCM1774 Exported Functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t PCM1774_RegisterBusIO(PCM1774_Object_t *pObj, PCM1774_IO_t *pIO)
{
  int32_t ret;

  if (pObj == NULL)
  {
    ret = PCM1774_ERROR;
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
      ret = PCM1774_ERROR;
    }
  }

  return ret;
}

/**
 * @brief  Initialize the PCM1774 sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t PCM1774_Init(PCM1774_Object_t *pObj, void *params)
{
  
  if (pObj->is_initialized == 0U)
  {
    if (pcm1774_Init(&pObj->Ctx, 20, (uint32_t) NULL) != PCM1774_OK)
    {
      return PCM1774_ERROR;
    }
  }

  pObj->is_initialized = 1;

  return PCM1774_OK;
}

/**
 * @brief  Deinitialize the PCM1774 sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t PCM1774_DeInit(PCM1774_Object_t *pObj)
{
  if (pObj->is_initialized == 1U)
  {
    pcm1774_DeInit(&(pObj->Ctx));
    pObj->is_initialized = 0;
  }
  
  return PCM1774_OK;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  pObj the device pObj
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t PCM1774_ReadID(PCM1774_Object_t *pObj, uint32_t *Id)
{
  if (pcm1774_ReadID(&(pObj->Ctx)) != PCM1774_OK)
  {
    return PCM1774_ERROR;
  }

  return PCM1774_OK;
}


int32_t PCM1774_Play(PCM1774_Object_t *pObj, uint16_t* pBuffer, uint16_t Size)
{
  if (pObj->audio_is_enabled != 0)
  {
    return PCM1774_ERROR;
  }
  else if (pcm1774_Play(&(pObj->Ctx), pBuffer, Size) != PCM1774_OK)
  {
    return PCM1774_ERROR;
  }
  else
  {
    pObj->audio_is_enabled = 1;
  }
  
  return PCM1774_OK;
}



int32_t PCM1774_Pause(PCM1774_Object_t *pObj)
{
  if (pObj->audio_is_enabled == 1)
  {
    pcm1774_Pause(&(pObj->Ctx));
    pObj->audio_is_enabled = 0;
  }

  return PCM1774_OK;
}



int32_t PCM1774_Resume(PCM1774_Object_t *pObj)
{
  if (pObj->audio_is_enabled != 1)
  {
    return PCM1774_ERROR;
  }
  else if (pcm1774_Resume(&(pObj->Ctx)) != PCM1774_OK)
  {
    return PCM1774_ERROR;
  }
  else
  {
    pObj->audio_is_enabled = 1;
  }

  return PCM1774_OK;
}



int32_t PCM1774_Stop(PCM1774_Object_t *pObj, uint32_t Cmd)
{
  if (pObj->audio_is_enabled != 0)
  {
    return PCM1774_ERROR;
  }
  else if (pcm1774_Stop(&(pObj->Ctx), Cmd) != PCM1774_OK)
  {
    return PCM1774_ERROR;
  }
  else
  {
    pObj->audio_is_enabled = 0;
  }

  return PCM1774_OK;
}



int32_t PCM1774_SetVolume(PCM1774_Object_t *pObj, uint32_t Cmd, uint8_t Volume)
{
  if (pcm1774_SetVolume(&(pObj->Ctx), Volume) != PCM1774_OK)
  {
    return PCM1774_ERROR;
  }

  return PCM1774_OK;
}



int32_t PCM1774_SetMute(PCM1774_Object_t *pObj, uint32_t Cmd)
{
  if (pcm1774_SetMute(&(pObj->Ctx), Cmd) != PCM1774_OK)
  {
    return PCM1774_ERROR;
  }

  return PCM1774_OK;
}



int32_t PCM1774_SetOutputMode(PCM1774_Object_t *pObj, uint8_t Output)
{
  pcm1774_SetOutputMode(&(pObj->Ctx), Output);
  return PCM1774_OK;
}



int32_t PCM1774_SetFrequency(PCM1774_Object_t *pObj, uint32_t AudioFreq)
{
  pcm1774_SetFrequency(&(pObj->Ctx), AudioFreq);
  return PCM1774_OK;
}



int32_t PCM1774_Reset(PCM1774_Object_t *pObj)
{
  pcm1774_Reset(&(pObj->Ctx));
  return PCM1774_OK;
}

int32_t PCM1774_SetResolution(void* a, uint32_t b)
{
  return PCM1774_OK;
}

int32_t PCM1774_GetResolution(void* a, uint32_t* b)
{
  return PCM1774_OK;
}

int32_t PCM1774_SetProtocol(void* a, uint32_t b)
{
  return PCM1774_OK;
}

int32_t PCM1774_GetProtocol(void* a, uint32_t* b)
{
  return PCM1774_OK;
}

int32_t PCM1774_GetVolume(void* a, uint32_t b, uint8_t* c)
{
  return PCM1774_OK;
}

int32_t PCM1774_GetFrequency(void* a)
{
  return PCM1774_OK;
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
  PCM1774_Object_t *pObj = (PCM1774_Object_t *)Handle;

  if (pObj->IO.BusType == (uint32_t)PCM1774_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte read */
    return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
  }
  else /* SPI 3-Wires */
  {
    /* Enable Multi-byte read */
    return PCM1774_ERROR;
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
  PCM1774_Object_t *pObj = (PCM1774_Object_t *)Handle;

  if (pObj->IO.BusType == (uint32_t)PCM1774_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte write */
    return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
  }
  else /* SPI 3-Wires */
  {
    /* Enable Multi-byte write */
    return PCM1774_ERROR;
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
