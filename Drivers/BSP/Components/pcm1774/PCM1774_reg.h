/**
******************************************************************************
* @file    PCM1774_CODEC_driver.c
* @author  SRA - Central Labs
* @brief   Definitions for the PCM1774_CODEC_driver.c firmware driver
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PCM1774_CODEC_DRIVER__H
#define __PCM1774_CODEC_DRIVER__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "audio.h"

/* Exported types ------------------------------------------------------------*/
extern AUDIO_Drv_t PCM1774_drv;


/** @addtogroup COMPONENTS
* @{
*/

/** @addtogroup PCM1774
* @{
*/

/** @defgroup PCM1774_Exported_Types PCM1774 Exported Types
* @{
*/

typedef enum 
{
  PCM1774_OK = (uint8_t)0, 
  PCM1774_ERROR = 1 , 
  PCM1774_NOT_IMPLEMENTED = 2
} 
PCM1774_Error_et;

typedef enum {
  PCM1774_CODEC_MUTE_ENABLE 		        = 0x40,
  PCM1774_CODEC_MUTE_DISABLE 		        = 0  	
}
PCM1774_CODEC_MUTE_t;


typedef int32_t (*PCM1774_write_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef int32_t (*PCM1774_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  PCM1774_write_ptr  write_reg;
  PCM1774_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} PCM1774_ctx_t;

    
/**
* @}
*/


/** @defgroup PCM1774_Exported_Constants PCM1774 Exported Constants
* @{
*/
/************** I2C Address *****************/

#define PCM1774_CODEC_I2C_ADDRESS_LOW   0x8C 
#define PCM1774_CODEC_I2C_ADDRESS_HIGH  0x8E 

/* Private Function Prototype -------------------------------------------------------*/

/************** Device Register  *******************/

#define PCM1774_CODEC_VOL_HPA_L                 0x40
#define PCM1774_CODEC_VOL_HPA_R                	0x41
#define PCM1774_CODEC_DAC_DIG_ATT_SM_L          0x44
#define PCM1774_CODEC_DAC_DIG_ATT_SM_R          0x45
#define PCM1774_CODEC_DAC_OS_DE_AI_DGC          0x46
#define PCM1774_CODEC_ANALOG_MIXER_PU_PD        0x48
#define PCM1774_CODEC_DAC_HPA_PU_PD             0x49
#define PCM1774_CODEC_ANALOG_OUT_CONFIG_SEL     0x4A
#define PCM1774_CODEC_HPA_INS_DETECTION_SP      0x4B
#define PCM1774_CODEC_SD_STATUS_READ_BACK       0x4D
#define PCM1774_CODEC_PG_1_2_5_6_PU_PD 	        0x52
#define PCM1774_CODEC_MASTER_MODE     	        0x54
#define PCM1774_CODEC_SYSRESET_FSC_DATASWAP     0x55
#define PCM1774_CODEC_BCK_CONF_FSC_ZC 	        0x56
#define PCM1774_CODEC_ANALOG_IN_SELECT          0x57
#define PCM1774_CODEC_ANALOG_MIX_SWITCH         0x58
#define PCM1774_CODEC_A_TO_A_PATH            	0x59
#define PCM1774_CODEC_MIC_BOOST          	0x5A
#define PCM1774_CODEC_BASS_BOOST_GAIN           0x5C
#define PCM1774_CODEC_MIDDLE_BOOST_GAIN         0x5D
#define PCM1774_CODEC_TREBLE_BOOST_GAIN         0x5E
#define PCM1774_CODEC_SOUND_EFFECT_SOURCE_3D    0x5F
#define PCM1774_CODEC_DIGITAL_MONO_MIX          0x60
#define PCM1774_CODEC_PG1_PG2_ADD_GAIN          0x7C
#define PCM1774_CODEC_PU_PD_TIME_CONTROL        0x7D
#define  	PCM1774_CODEC_MUTE_MASK  	0x40

/**
* @}
*/

/** @defgroup PCM1774_Exported_Function_Prototypes PCM1774 Exported Function Prototypes
* @{
*/
int32_t pcm1774_Init(PCM1774_ctx_t *handle, uint8_t Volume, uint32_t AudioFreq);
int32_t pcm1774_DeInit(PCM1774_ctx_t *handle);
int32_t pcm1774_ReadID(PCM1774_ctx_t *handle);
int32_t pcm1774_Play(PCM1774_ctx_t *handle, uint16_t* pBuffer, uint16_t Size);
int32_t pcm1774_Pause(PCM1774_ctx_t *handle);
int32_t pcm1774_Resume(PCM1774_ctx_t *handle);
int32_t pcm1774_Stop(PCM1774_ctx_t *handle, uint32_t Cmd);
int32_t pcm1774_SetVolume(PCM1774_ctx_t *handle, uint8_t Volume);
int32_t pcm1774_SetMute(PCM1774_ctx_t *handle, uint32_t Cmd);
int32_t pcm1774_SetOutputMode(PCM1774_ctx_t *handle, uint8_t Output);
int32_t pcm1774_SetFrequency(PCM1774_ctx_t *handle, uint32_t AudioFreq);
int32_t pcm1774_Reset(PCM1774_ctx_t *handle);
/**
* @}
*/

/**
* @}
*/

/**
* @}
*/



#endif
