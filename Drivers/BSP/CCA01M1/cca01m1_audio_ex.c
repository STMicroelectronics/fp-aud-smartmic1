/**
  ******************************************************************************
  * @file    cca01m1_audio_ex.c
  * @author  SRA
  * @brief   This file provides a set of extended functions needed to manage the 
  *          sound terminal device.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "cca01m1_audio_ex.h"

extern void *CompObj;
/** @addtogroup BSP BSP
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1 X_NUCLEO_CCA01M1
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1_AUDIO X_NUCLEO_CCA01M1_AUDIO
* @{
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_Defines Private Defines
* @{
*/
#ifndef NULL
#define NULL      (void *) 0
#endif
/**
* @}
*/

/**
* @brief  This function can be used to set advanced DSP options in order to 
*         use advanced features on the STA350BW device.
* @param  handle: device handle
* @param  option: specific option to be setted up
*         This parameter can be a value of @ref STA350BW_DSP_option_selection 
* @param  state: state of the option to be controlled. Depending on the selected 
*         DSP feature to be controlled, this value can be either ENABLE/DISABLE 
*         or a specific numerical parameter related to the specific DSP function. 
*         This parameter can be a value of @ref STA350BW_state_define   
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_SetDSPOption(uint32_t Instance, uint8_t option, uint8_t state)
{  
  int32_t ret = BSP_ERROR_NONE;
  
  if(STA350BW_SetDSPOption(CompObj, option, state) != 0)
  { 
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }    
  
  return ret;  
}


/**
* @brief  Set Equalization.
* @param  handle: device handle
* @param  ramBlock: device RAM block to be written
* @param  filterNumber: filter to be used
* @param  filterValues: pointer to filter values    
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
* @note   for specific information about biquadratic filters setup, please refer 
*         to the STA350BW component datasheet, available at 
*         http://www.st.com/web/catalog/sense_power/FM125/SC1756/PF251568?s_searchtype=partnumber
*/
int32_t CCA01M1_AUDIO_OUT_SetEq(uint32_t Instance, uint8_t ramBlock, uint8_t filterNumber, uint32_t * filterValues)
{    
  int32_t ret = BSP_ERROR_NONE;
  
  if(STA350BW_SetEq(CompObj, ramBlock, filterNumber, filterValues) != 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  return ret;    
}

/**
* @brief  Set Tone.
* @param  handle: device handle
* @retval COMPONENT_OK if no problem during execution, COMPONENT_ERROR otherwise
*/
int32_t CCA01M1_AUDIO_OUT_SetTone(uint32_t Instance, uint8_t toneGain)
{
  int32_t ret = BSP_ERROR_NONE;
  if(STA350BW_SetTone(CompObj, toneGain) != 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
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

