/**
******************************************************************************
* @file    acoustic_bf.h
* @author  SRA
* @version v3.0.0
* @date    8-Oct-2021
* @brief   This file contains Acoustic Echo Cancellation library definitions.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACOUSTIC_EC_H
#define __ACOUSTIC_EC_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Exported types ------------------------------------------------------------*/

/** @addtogroup MIDDLEWARES
* @{
*/

/** @defgroup ACOUSTIC_EC ACOUSTIC_EC
* @{
*/

/** @defgroup Acoustic_EC_Exported_Constants AcousticEC Exported Constants
* @{/  
*/

/** @defgroup Acoustic_EC_errors
* @brief    Echo cancellation errors 
* @{
*/ 
#define ACOUSTIC_EC_TAIL_LENGTH_ERROR                  	((uint32_t)0x00000001)
#define ACOUSTIC_EC_AEC_LEVEL_ERROR                    	((uint32_t)0x00000002)
#define ACOUSTIC_EC_PTR_CHANNELS_ERROR                 	((uint32_t)0x00000004)
#define ACOUSTIC_EC_PREPROCESS_ERROR                   	((uint32_t)0x00000010)

#ifndef ACOUSTIC_LOCK_ERROR
#define ACOUSTIC_LOCK_ERROR                         	((uint32_t)0x10000000)
#endif 
/**
* @}
*/

/** @defgroup Acoustic_EC_preprocessor
* @brief    Echo cancellation preprocessor 
* @{
*/ 
#define ACOUSTIC_EC_PREPROCESS_ENABLE                   ((uint32_t)0x00000001)
#define ACOUSTIC_EC_PREPROCESS_DISABLE                  ((uint32_t)0x00000000)

/**
* @}
*/

/**
* @}
*/

/** @defgroup Acoustic_EC_Exported_Types AcousticEC Exported Types
* @{
*/
/**
* @brief  Library handler. It keeps track of the static parameters
*         and it handles the internal state of the algorithm.
*/
typedef struct
{
  uint16_t tail_length;                              /*!< Specifies the length of the filter tail. Default Value is 512. */
  uint32_t preprocess_init;                          /*!< Specifies the option for the preprocessor uinitialization. This parameter can be a value of @ref Acoustic_EC_preprocessor.  */  
  uint16_t ptr_primary_channels;                     /*!< Specifies the number of channel in the Primary Stream. Default Value is 1. */ 
  uint16_t ptr_reference_channels;                   /*!< Specifies the number of channel in the Reference Stream. Default Value is 1. */ 
  uint16_t ptr_output_channels;                      /*!< Specifies the number of channel in the Output Stream. Default Value is 1. */  
  uint32_t internal_memory_size;                     /*!< Keeps track of the amount of memory required for the current setup.
  It's filled by the libSpeexAEC_getMemorySize() function and must be
  used to allocate the right amount of RAM */
  uint32_t * pInternalMemory;                        /*!< Pointer to the memory allocated by the user */
  
} 
AcousticEC_Handler_t;

/**
* @brief  Library dynamic configuration handler. It contains dynamic parameters.
*/
typedef struct
{
  uint32_t preprocess_state;                         /*!< Enable or disable pre-process function */
  uint32_t AGC_value;                                /*!< Specifies the threshold for the AGC, if activated */
  uint32_t residual_echo_remove;                     /*!< Activate residual echo removal */   
  int32_t noise_suppress_default;                    /*!< Specifies the noise suppress default parameter of the preprocessor */    
  int32_t echo_suppress_default;                     /*!< Specifies the echo suppress default parameter of the preprocessor */ 
  int32_t echo_suppress_active;                      /*!< Specifies the echo suppress active parameter of the preprocessor */ 
} 
AcousticEC_Config_t;

/**
* @}
*/

/** @defgroup Acoustic_EC_Exported_Functions AcousticBF Exported Functions
* @{
*/

/**
* @brief  Fills the "internal_memory_size" of the pHandler parameter passed as argument with a value representing the
*         right amount of memory needed by the library, depending on the specific static parameters adopted.
* @param  pHandler: AcousticEC_Handler_t filled with desired parameters.
* @retval 0 if everything is fine.
*/
uint32_t AcousticEC_getMemorySize(AcousticEC_Handler_t * pHandler);

/**
* @brief  Library initialization
* @param  pHandler: AcousticEC_Handler_t filled with desired parameters.
* @retval 0 if everything is fine.
*         different from 0 if erroneous parameters have been passed to the Init function and the default value has been used.
*         The specific error can be recognized by checking the relative bit in the returned word.
*/
uint32_t AcousticEC_Init(AcousticEC_Handler_t * pHandler);

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
uint32_t AcousticEC_Data_Input(void *ptrPrimary, void *ptrReference, void *ptrBufferOut, AcousticEC_Handler_t * pHandler);

/**
* @brief  Library run function, performs audio analysis when all required data has been collected.
* @param  pHandler: pointer to the handler of the current SpeexAEC instance running
* @retval 0 if everything is ok, 1 otherwise.
*/
uint32_t AcousticEC_Process(AcousticEC_Handler_t * pHandler);

/**
* @brief  Library setup function, it sets the values for dynamic parameters.
*         It can be called at runtime to change dynamic parameters.
* @param  pHandler: AcousticEC_Handler_t filled with desired parameters.
* @param  pConfig: pointer to the dynamic parameters handler that will be filled with the current library configuration.
* @retval 0 if everything is ok, 1 otherwise.
*/
uint32_t AcousticEC_setConfig(AcousticEC_Handler_t * pHandler, AcousticEC_Config_t * pConfig);

/**
* @brief  Fills the pConfig structure with the actual dynamic parameters as they are used inside the library.
* @param  pHandler: AcousticEC_Handler_t filled with desired parameters.
* @param  pConfig: pointer to the dynamic parameters handler that will be filled with the current library configuration.
* @retval 0 if everything is ok, 1 otherwise.
*/
uint32_t AcousticEC_getConfig(AcousticEC_Handler_t * pHandler, AcousticEC_Config_t * pConfig);

/**
* @brief  To be used to retrieve version information.
* @param  none
* @retval Version Number.
*/
uint32_t AcousticEC_GetLibVersion(char *version);



/**
* @}
*/

/**
* @}
*/

/**
* @}
*/
#endif  /*__ACOUSTIC_EC_H*/

