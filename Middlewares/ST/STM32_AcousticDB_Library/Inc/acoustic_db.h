/**
******************************************************************************
* @file    acoustic_db.h
* @author  Central Labs
* @version V1.0.0
* @date    1-Sep-2016
* @brief   This file contains  dB SPL library definitions.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Some of the library code is based on the CMSIS DSP software library by ARM,
* a suite of common signal processing functions for use on Cortex-M processor
* based devices. Licencing terms are available in the attached release_note.html
* file, in the next lines of this
* document and it's available on the web at:
* http://www.keil.com/pack/doc/CMSIS/DSP/html/index.html
*
*   ARM licence note:
*
* Copyright (C) 2009-2012 ARM Limited.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*  - Neither the name of ARM nor the names of its contributors may be used
*   to endorse or promote products derived from this software without
*   specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACOUSTIC_DB_H
#define __ACOUSTIC_DB_H

#include "stdint.h"

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup ACOUSTIC_DB ACOUSTIC_DB
  * @{
  */

  /** @defgroup Acoustic_DB_Exported_Constants AcousticDB Exported Constants
* @{/  
*/

/** @defgroup Acoustic_DB_errors
* @brief    dB SL estimation errors 
* @{
*/ 
#define ACOUSTIC_DB_PTR_CHANNELS_ERROR             0x00000001
#define ACOUSTIC_DB_SAMPLING_FREQ_ERROR            0x00000002
/**
* @}
*/

/** @defgroup Acoustic_DB_Exported_Types AcousticDB Exported Types
* @{
*/
/**
 * @brief  Library handler. It keeps track of the static parameters
 *         and it handles the internal state of the algorithm.
 */
typedef struct
{
  uint32_t sampling_frequency;                  /*!< Specifies the sampling frequency*/  
  uint8_t in_ptr_channels;                      /*!< Number of channels in the stream of Microphone 1 */  
  uint32_t pInternalMemory[8];                  /*!< Pointer to the internal memory*/
  
} 
AcousticDB_Handler_t;

/**
 * @brief  Library dynamic configuration handler. It contains dynamic parameters.
 */
typedef struct
{
  int16_t offset;                           /*!< Specifies energy value below which the algorithm does not act */
} 
AcousticDB_Config_t;

/**
  * @}
  */

    /** @defgroup Acoustic_DB_Exported_Functions AcousticDB Exported Functions
 * @{
 */
 
/**
 * @brief  Library initialization.
 * @param  pHandler: AcousticDB_Handler_t filled with desired parameters.
 * @retval 0 if everything is fine.
 *         different from 0 if erroneous parameters have been passed to the Init function and the default value has been used.
 *         The specific error can be recognized by checking the relative bit in the returned word.
 */
uint32_t AcousticDB_Init(AcousticDB_Handler_t * pHandler);

/**
 * @brief  Library data input
 * @param  pInput: pointer to an array that contains PCM samples 
 * @param  nSamples: samples number to be processed
 * @param  pHandler: pointer to the handler of the curent dBSPL estimation instance running.
 * @retval 1 if data collection is finished and AcousticDB_Process must be called, 0 otherwise.
*/
uint32_t AcousticDB_Data_Input(void *pInput, uint32_t nSamples, AcousticDB_Handler_t * pHandler);

/**
 * @brief  Library run function, performs audio analysis when all required data has been collected.
 * @param  dB_Value: pointer to the int32_t variable that will contain the computed value.
 * @param  pHandler: pointer to the handler of the current dBSPL estimation instance running.
 * @retval 0 if everything is ok, 1 otherwise
*/
uint32_t AcousticDB_Process(int32_t * dB_Value, AcousticDB_Handler_t * pHandler);

/**
 * @brief  Library setup function, it sets the values for offset. It can be called at runtime to change
 *         dynamic parameters.
  * @retval 0 if everything is fine.
 *         different from 0 if erroneous parameters have been passed to the Init function and the default value has been used.
 *         The specific error can be recognized by checking the relative bit in the returned word.
*/
uint32_t AcousticDB_setConfig(AcousticDB_Handler_t * pHandler, AcousticDB_Config_t * pConfig);

/**
 * @brief  Fills the pConfig structure with the actual dynamic parameters as they are currently used inside the library.
 * @param  pHandler: pointer to the handler of the current library instance running.
 * @param  pConfig: pointer to the dynamic parameters handler that will be filled with the current library configuration
 * @retval 0 if everything is fine.
*/
uint32_t AcousticDB_getConfig(AcousticDB_Handler_t * pHandler, AcousticDB_Config_t * pConfig);

/**
 * @brief  To be used to retrieve version information.
 * @param  version char array to be filled with the current library version
 * @retval 0 if everything is fine.
*/
uint32_t AcousticDB_GetLibVersion(char *version);

/**
  * @}
  */

/**
* @}
*/
  
/**
  * @}
  */

#endif /* __ACOUSTIC_DB_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
