/**
******************************************************************************
* @file    acousticEC.c
* @author  SRA
* @brief   Acoustic Echo Cancellation APIs
******************************************************************************
* @attention
*
* Copyright (c) 2022 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file in
* the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*                        
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <arm_math.h>
#include "Echo_library.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

/** @addtogroup MIDDLEWARES
* @{
*/

/** @defgroup AcousticEC AcousticEC
* @{
*/

/** @defgroup AcousticEC_Exported_Functions AcousticEC Exported Functions
* @{
*/

/**
* @brief  Library initialization
* @param  pHandler: AcousticEC_Handler_t filled with desired parameters.
* @retval 0 if everything is fine.
*         different from 0 if erroneous parameters have been passed to the Init function and the default value has been used.
*         The specific error can be recognized by checking the relative bit in the returned word.
*/
uint32_t AcousticEC_Init(AcousticEC_Handler_t *pHandler)
{
  return libSpeexAEC_Init(pHandler);
}

/**
* @brief  Library data input and output function.
* @param  ptrPrimary: pointer to an array that contains PCM samples (16 bit signed int)
*         of the primary signal (1 ms = 16 samples at 16 KHZ).
* @param  ptrReference: pointer to an array that contains PCM samples (16 bit signed int)
*         of the reference (echo) signal (1 ms = 16 samples at 16 KHZ).
* @param  ptrBufferOut: pointer to an array that will contain output PCM samples
*         (1 ms for  processed channel = 16 samples at 16 KHZ).
* @param  pHandler: pointer to the handler of the current SpeexAEC instance running.
* @retval 1 if data collection is finished and libSpeexAEC_Process must be called, 0 otherwise.
* @note   Input/output function reads and write samples skipping the required number of values depending on the
*         ptr_Mx_channels configuration
*/
uint32_t AcousticEC_Data_Input(void *ptrPrimary, void *ptrReference, void *ptrBufferOut, AcousticEC_Handler_t *pHandler)
{
  return libSpeexAEC_Data_Input(ptrPrimary, ptrReference, ptrBufferOut, pHandler);
}

/**
* @brief  Library run function, performs audio analysis when all required data has been collected.
* @param  pHandler: pointer to the handler of the current SpeexAEC instance running
* @retval 0 if everything is ok, 1 otherwise.
*/
uint32_t AcousticEC_Process(AcousticEC_Handler_t *pHandler)
{
  return libSpeexAEC_Process(pHandler);
}

/**
* @brief  Library setup function, it sets the values for dynamic parameters.
*         It can be called at runtime to change dynamic parameters.
* @param  pHandler: AcousticEC_Handler_t filled with desired parameters.
* @param  pConfig: pointer to the dynamic parameters handler that will be filled with the current library configuration.
* @retval 0 if everything is ok, 1 otherwise.
*/
uint32_t AcousticEC_setConfig(AcousticEC_Handler_t *pHandler, AcousticEC_Config_t *pConfig)
{
  return libSpeexAEC_setConfig(pHandler, pConfig);
}

/**
* @brief  Fills the pConfig structure with the actual dynamic parameters as they are used inside the library.
* @param  pHandler: AcousticEC_Handler_t filled with desired parameters.
* @param  pConfig: pointer to the dynamic parameters handler that will be filled with the current library configuration.
* @retval 0 if everything is ok, 1 otherwise.
*/
uint32_t AcousticEC_getConfig(AcousticEC_Handler_t *pHandler, AcousticEC_Config_t *pConfig)
{
  return libSpeexAEC_getConfig(pHandler, pConfig);
}

/**
* @brief  Fills the "internal_memory_size" of the pHandler parameter passed as argument with a value representing the
*         right amount of memory needed by the library, depending on the specific static parameters adopted.
* @param  pHandler: AcousticEC_Handler_t filled with desired parameters.
* @retval 0 if everything is fine.
*/
uint32_t AcousticEC_getMemorySize(AcousticEC_Handler_t *pHandler)
{
  return libSpeexAEC_getMemorySize(pHandler);
}

/**
* @brief  To be used to retrieve version information.
* @param  pHandler: AcousticEC_Handler_t filled with desired parameters.
* @retval Version Number.
*/
uint32_t AcousticEC_GetLibVersion(char *version)
{
  return libSpeexAEC_GetLibVersion(version);
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

