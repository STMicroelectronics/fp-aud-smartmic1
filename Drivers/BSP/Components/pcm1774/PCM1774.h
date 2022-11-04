/**
 ******************************************************************************
 * @file    pcm1774.h
 * @author  SRA - Central Labs
 * @brief   PCM1774 header driver file
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PCM1774_H
#define PCM1774_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "PCM1774_reg.h"
#include <string.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @addtogroup PCM1774 PCM1774
 * @{
 */

/** @defgroup PCM1774_Exported_Types PCM1774 Exported Types
 * @{
 */

typedef int32_t (*PCM1774_Init_Func)(void);
typedef int32_t (*PCM1774_DeInit_Func)(void);
typedef int32_t (*PCM1774_GetTick_Func)(void);
typedef int32_t (*PCM1774_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*PCM1774_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  PCM1774_Init_Func          Init;
  PCM1774_DeInit_Func        DeInit;
  uint32_t                  BusType; /*0 means I2C, 1 means SPI-3-Wires */
  uint8_t                   Address;
  PCM1774_WriteReg_Func      WriteReg;
  PCM1774_ReadReg_Func       ReadReg;
  PCM1774_GetTick_Func       GetTick;
} PCM1774_IO_t;

typedef struct
{
  PCM1774_IO_t        IO;
  PCM1774_ctx_t       Ctx;
  uint8_t            is_initialized;
  uint8_t            audio_is_enabled;
} PCM1774_Object_t;

typedef struct
{
  int32_t (*Init)(PCM1774_Object_t *, void *);
  int32_t (*DeInit)(PCM1774_Object_t *);
  int32_t (*ReadID)(PCM1774_Object_t *, uint32_t *);
  
  int32_t (*Play)(PCM1774_Object_t *, uint16_t*, uint16_t);
  int32_t (*Pause)(PCM1774_Object_t *);
  int32_t (*Resume)(PCM1774_Object_t *);
  int32_t (*Stop)(PCM1774_Object_t *, uint32_t);
  int32_t (*SetFrequency)(PCM1774_Object_t *, uint32_t);
    int32_t  (*GetFrequency   )(void*);  
  int32_t (*SetVolume)(PCM1774_Object_t *, uint32_t, uint8_t);
    int32_t  (*GetVolume      )(void*, uint32_t, uint8_t*);
  int32_t (*SetMute)(PCM1774_Object_t *, uint32_t);  
  int32_t (*SetOutputMode)(PCM1774_Object_t *, uint8_t);
    int32_t  (*SetResolution  )(void*, uint32_t);
    int32_t  (*GetResolution  )(void*, uint32_t*);  
    int32_t  (*SetProtocol    )(void*, uint32_t);
    int32_t  (*GetProtocol    )(void*, uint32_t*);
  int32_t (*Reset)(PCM1774_Object_t *);
} PCM1774_AUDIO_Drv_t;


/**
 * @}
 */

/** @defgroup PCM1774_Exported_Constants PCM1774 Exported Constants
 * @{
 */
#define PCM1774_I2C_BUS           0U

/** PCM1774 error codes  **/
#define PCM1774_OK                 0
#define PCM1774_ERROR             -1

/**
 * @}
 */

/** @addtogroup PCM1774_Exported_Functions PCM1774 Exported Functions
 * @{
 */

int32_t PCM1774_RegisterBusIO(PCM1774_Object_t *pObj, PCM1774_IO_t *pIO);
int32_t PCM1774_Init(PCM1774_Object_t *pObj, void *params);
int32_t PCM1774_DeInit(PCM1774_Object_t *pObj);
int32_t PCM1774_ReadID(PCM1774_Object_t *pObj, uint32_t *Id);

int32_t PCM1774_Play(PCM1774_Object_t *pObj, uint16_t* pBuffer, uint16_t Size);
int32_t PCM1774_Pause(PCM1774_Object_t *pObj);
int32_t PCM1774_Resume(PCM1774_Object_t *pObj);
int32_t PCM1774_Stop(PCM1774_Object_t *pObj, uint32_t Cmd);
int32_t PCM1774_SetVolume(PCM1774_Object_t *pObj, uint32_t Cmd, uint8_t Volume);
int32_t PCM1774_SetMute(PCM1774_Object_t *pObj, uint32_t Cmd);
int32_t PCM1774_SetOutputMode(PCM1774_Object_t *pObj, uint8_t Output);
int32_t PCM1774_SetFrequency(PCM1774_Object_t *pObj, uint32_t AudioFreq);
int32_t PCM1774_Reset(PCM1774_Object_t *pObj);
int32_t PCM1774_SetResolution(void* a, uint32_t b);
int32_t PCM1774_GetResolution(void* a, uint32_t* b);  
int32_t PCM1774_SetProtocol(void* a, uint32_t b);
int32_t PCM1774_GetProtocol(void* a, uint32_t* b);
int32_t PCM1774_GetVolume(void* a, uint32_t b, uint8_t* c);
int32_t PCM1774_GetFrequency(void* a);  

int32_t PCM1774_Read_Reg(PCM1774_Object_t *pObj, uint8_t Reg, uint8_t *Data);
int32_t PCM1774_Write_Reg(PCM1774_Object_t *pObj, uint8_t Reg, uint8_t Data);

/**
 * @}
 */

/** @addtogroup PCM1774_Exported_Variables PCM1774 Exported Variables
 * @{
 */

extern PCM1774_AUDIO_Drv_t PCM1774_AUDIO_Driver;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

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
