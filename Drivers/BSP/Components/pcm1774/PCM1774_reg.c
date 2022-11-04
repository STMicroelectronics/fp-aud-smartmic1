/**
 ******************************************************************************
 * @file    PCM1774_reg.c
 * @author  SRA - Central Labs
 * @brief   This file provides the PCM1774 audio driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "PCM1774_reg.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup BSP
* @{
*/

/** @addtogroup COMPONENTS
* @{
*/

/** @addtogroup PCM1774
* @{
*/

/** @defgroup PCM1774_Private_Types PCM1774 Private Types
* @{
*/

/**
* @}
*/

/** @defgroup PCM1774_Private_Defines PCM1774 Private Defines
* @{
*/

/**
* @}
*/

/** @defgroup PCM1774_Private_Macros PCM1774 Private Macros
* @{
*/

/**
* @}
*/

/** @defgroup PCM1774_Private_Variables PCM1774 Private Variables
* @{
*/


/**
* @}
*/

/** @defgroup PCM1774_Imported_Functions PCM1774 Imported Functions 
* @{
*/

/**
* @}
*/

/** @defgroup PCM1774_Private_Function_Prototypes PCM1774 Private Function Prototypes
* @{
*/
/* Private function prototypes -----------------------------------------------*/
int32_t PCM1774_ReadReg(PCM1774_ctx_t *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
int32_t PCM1774_WriteReg(PCM1774_ctx_t *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
/**
* @}
*/


/** @defgroup PCM1774_Exported_Functions PCM1774 Exported Functions
* @{
*/

/**
* @brief        Initialize the device.
* @param        Volume Initial Volume.
* @param        AudioFreq Sampling frequency.
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_Init(PCM1774_ctx_t *handle, uint8_t Volume, uint32_t AudioFreq)
{
  uint8_t tmp = Volume;  
  PCM1774_WriteReg(handle, PCM1774_CODEC_VOL_HPA_L, 1, &tmp);    
  PCM1774_WriteReg(handle, PCM1774_CODEC_VOL_HPA_R, 1, &tmp);
    
  tmp = 0x3F;  
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_L, 1, &tmp);
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_R, 1, &tmp);       
  
  tmp = 0x01; 
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_OS_DE_AI_DGC, 1, &tmp);
  
  tmp = 0xE0; 
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_HPA_PU_PD, 1, &tmp);
  
  tmp = 0x01; 
  PCM1774_WriteReg(handle, PCM1774_CODEC_BCK_CONF_FSC_ZC, 1, &tmp);    
  
  tmp = 0x03; 
  PCM1774_WriteReg(handle, PCM1774_CODEC_ANALOG_MIXER_PU_PD, 1, &tmp);
    
  tmp = 0x11; 
  PCM1774_WriteReg(handle, PCM1774_CODEC_ANALOG_MIX_SWITCH, 1, &tmp);
    
  tmp = 0xEC; 
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_HPA_PU_PD, 1, &tmp);
    
  tmp = 0x01; 
  PCM1774_WriteReg(handle, PCM1774_CODEC_ANALOG_OUT_CONFIG_SEL, 1, &tmp);
    
  tmp = 0x30; 
  PCM1774_WriteReg(handle, PCM1774_CODEC_PG_1_2_5_6_PU_PD, 1, &tmp);
    
  return PCM1774_OK;   
}

/**
* @brief        Deinitialize the device.
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_DeInit(PCM1774_ctx_t *handle)
{  
  return PCM1774_NOT_IMPLEMENTED; 
}
 
/**
* @brief        Read device ID.
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
 int32_t pcm1774_ReadID(PCM1774_ctx_t *handle)
 {  
   return PCM1774_NOT_IMPLEMENTED;   
 }

/**
* @brief        Play an audio buffer
* @param        *pBuffer buffer to be played.
* @param        Size Buffer size.
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_Play(PCM1774_ctx_t *handle,uint16_t* pBuffer, uint16_t Size)
{  
  uint8_t tmp;
  PCM1774_ReadReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_L, 1, &tmp);
    
  tmp &= ~PCM1774_CODEC_MUTE_MASK;  
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_L, 1, &tmp);
  PCM1774_ReadReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_R, 1, &tmp);
    
  tmp &= ~PCM1774_CODEC_MUTE_MASK;
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_R, 1, &tmp);
    
  return PCM1774_OK;     
}

/**
* @brief        Pause device.
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_Pause(PCM1774_ctx_t *handle)
{
  return PCM1774_NOT_IMPLEMENTED;     
}

/**
* @brief        Resume device.
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_Resume(PCM1774_ctx_t *handle)
{
  uint8_t tmp;
  
  PCM1774_ReadReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_L, 1, &tmp);
  
  tmp &= ~PCM1774_CODEC_MUTE_MASK;
  tmp |= 1 << 6;
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_L, 1, &tmp);
  PCM1774_ReadReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_R, 1, &tmp);
    
  tmp &= ~PCM1774_CODEC_MUTE_MASK;
  tmp |= 1 << 6;
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_R, 1, &tmp);
  
  return PCM1774_OK;   
}

/**
* @brief        Stop the device.
* @param        Cmd Command
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_Stop(PCM1774_ctx_t *handle, uint32_t Cmd)
{
  uint8_t tmp;
  
  PCM1774_ReadReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_L, 1, &tmp);
  
  tmp &= ~PCM1774_CODEC_MUTE_MASK;
  tmp |= 1 << 6;
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_L, 1, &tmp);
  PCM1774_ReadReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_R, 1, &tmp);
  
  tmp &= ~PCM1774_CODEC_MUTE_MASK;
  tmp |= 1 << 6;
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_R, 1, &tmp);

  return PCM1774_OK;   
}

/**
* @brief        Set the current volume
* @param        Volume Volume
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_SetVolume(PCM1774_ctx_t *handle, uint8_t Volume)
{
  uint8_t tmp = Volume;  
  PCM1774_WriteReg(handle, PCM1774_CODEC_VOL_HPA_L, 1, &tmp);
  PCM1774_WriteReg(handle, PCM1774_CODEC_VOL_HPA_R, 1, &tmp);

  return PCM1774_OK;
}

/**
* @brief        Set mute.
* @param        Cmd Command
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_SetMute(PCM1774_ctx_t *handle, uint32_t Cmd)
{  
  uint8_t tmp;
  PCM1774_ReadReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_L, 1, &tmp);

  tmp &= ~PCM1774_CODEC_MUTE_MASK;
  tmp |= Cmd << 6;
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_L, 1, &tmp);
  PCM1774_ReadReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_R, 1, &tmp);

  tmp &= ~PCM1774_CODEC_MUTE_MASK;
  tmp |= Cmd << 6;
  PCM1774_WriteReg(handle, PCM1774_CODEC_DAC_DIG_ATT_SM_R, 1, &tmp);
  
  return PCM1774_OK;  
}

/**
* @brief        Set Out mode.
* @param        Output Output mode
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_SetOutputMode(PCM1774_ctx_t *handle, uint8_t Output)
{
  return PCM1774_NOT_IMPLEMENTED;   
}

/**
* @brief        Set the desired frequency.
* @param        AudioFreq Frequency
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_SetFrequency(PCM1774_ctx_t *handle, uint32_t AudioFreq)
{
  return PCM1774_NOT_IMPLEMENTED;   
}

/**
* @brief        Reset device.
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t pcm1774_Reset(PCM1774_ctx_t *handle)
{
  return PCM1774_NOT_IMPLEMENTED;   
}

/**
* @}
*/

/** @defgroup PCM1774_Private_Functions PCM1774 Private Functions
* @{
*/

/**
* @brief        Generic reading function. It must be fullfilled with either I2C 
*               or SPI writing function.
* @param        RegAddr address of the register.
* @param        NumByteToRead Size to be read.
* @param        *Data buffer to be filled with read data.
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t PCM1774_ReadReg(PCM1774_ctx_t *ctx, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
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
* @retval       PCM1774_OK if correct setup, PCM1774_ERROR otherwise
*/
int32_t PCM1774_WriteReg(PCM1774_ctx_t *ctx, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data )
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
