/**
******************************************************************************
* @file    AcousticBF.c
* @author  SRA
* @brief   Acoustic BeamForming APIs
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
#include "libBeamforming.c"

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

/** @defgroup AcousticBF AcousticBF
* @{
*/

/** @defgroup AcousticBF_Exported_Functions AcousticBF Exported Functions
* @{
*/

/**
* @brief  Library initialization
* @param  pHandler: AcousticBF_Handler_t filled with desired parameters.
* @retval 0 if everything is fine.
*         different from 0 if erroneous parameters have been passed to the Init function and the default value has been used.
*         The specific error can be recognized by checking the relative bit in the returned word.
*/
uint32_t AcousticBF_Init(AcousticBF_Handler_t * pHandler)
{
  return libBeamforming_Init(pHandler); 
}

/**
* @brief  Library data input/output
* @param  pM1: pointer to an array that contains PCM or PDM samples of the first channel (1 millisecond).
* @param  pM2: pointer to an array that contains PCM or PDM samples of the second channel (1 millisecond).
* @param  ptr_Out: pointer to an array that will contain PCM samples of output data (1 millisecond). If the reference channel
*         is activated, two channels will be written in the output array.
* @param  pHandler: pointer to the handler of the current Beamforming instance running.
* @retval 1 if data collection is finished and Beamforming_SecondStep must be called, 0 otherwise.
* @note   Input/output function reads and write samples skipping the required number of values depending on the
*         ptr_Mx_channels configuration.
*/
uint32_t AcousticBF_FirstStep(void *pM1, void *pM2, void *ptr_Out, AcousticBF_Handler_t * pHandler)
{
  return libBeamforming_FirstStep(pM1, pM2, ptr_Out, pHandler); 
}

/**
* @brief  Library run function, performs audio analysis when all required data has been collected.
* @param  pHandler: pointer to the handler of the current beamforming instance running.
* @retval 0 if everything is ok.
*/
uint32_t AcousticBF_SecondStep(AcousticBF_Handler_t * pHandler)
{  
  return libBeamforming_SecondStep(pHandler);
}

/**
* @brief  Library setup function, it sets the values for dynamic parameters. It can be called at runtime to change
*         dynamic parameters.
* @param  pHandler: pointer to the handler of the current Beamforming instance running.
* @param  pConfig: pointer to the dynamic parameters handler containing the new library configuration.
* @retval 0 if everything is fine.
*         different from 0 if erroneous parameters have been passed to the setConfig function and the default
*         value has been used. The specific error can be recognized by checking the relative bit in the returned word.
*/
uint32_t AcousticBF_setConfig(AcousticBF_Handler_t * pHandler, AcousticBF_Config_t * pConfig)
{
  return libBeamforming_setConfig(pHandler, pConfig);
}

/**
* @brief  Fills the pConfig structure with the actual dynamic parameters as they are used inside the library.
* @param  pHandler: pointer to the handler of the current Beamforming instance running.
* @param  pConfig: pointer to the dynamic parameters handler that will be filled with the current library configuration.
* @retval 0 if everything is fine.
*/
uint32_t AcousticBF_getConfig(AcousticBF_Handler_t * pHandler, AcousticBF_Config_t * pConfig)
{  
  return libBeamforming_getConfig(pHandler, pConfig);
}

/**
* @brief  Fills the "internal_memory_size" of the pHandler parameter passed as argument with a value representing the
*         right amount of memory needed by the library, depending on the specific static parameters adopted.
* @param  pHandler: libBeamforming_Handler filled with desired parameters.
* @retval 0 if everything is fine.
*/
uint32_t AcousticBF_getMemorySize(AcousticBF_Handler_t * pHandler)
{
  return libBeamforming_getMemorySize(pHandler);
}

/**
* @brief  To be used to retrieve version information.
* @param  version char array to be filled with the current library version
* @retval 0 if everything is fine.
*/
uint32_t AcousticBF_GetLibVersion(char *version)
{
  return libBeamforming_GetLibVersion(version);
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

