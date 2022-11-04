/**
******************************************************************************
* @file    audio_application.h 
* @author  SRA
* 
* 
* @brief   Header for prox_application.c module.
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
#ifndef __PROX_APPLICATION_H
#define __PROX_APPLICATION_H

/* Includes ------------------------------------------------------------------*/
#include "BlueCoin_prox.h"
#include "tof_gestures.h"
#include "tof_gestures_TAP_1.h"
#include "tof_gestures_SWIPE_1.h"
#include "tof_gestures_DIRSWIPE_1.h"
#include <limits.h>

#include "Audio_SerialCmd_Handlers.h"
#include "audio_application.h"




/** @addtogroup SMARTMIC1 
* @{
*/

/** @defgroup SMARTMIC1_PROX 
* @{
*/ 

/** @defgroup SMARTMIC1_PROX_Private_Types 
* @{
*/  



/** @defgroup SMARTMIC1_PROX_Exported_Defines 
* @{
*/
  
#define START_M 0
#define WAIT_M 1
#define GET_M 2

/**
* @}
*/


/** @defgroup SMARTMIC1_PROX_Exported_Functions 
* @{
*/
void TOF_Init(void);
void PeriodicTOF(void);



/**
* @}
*/

/**
* @}
*/

/**
* @}
*/



#endif /* __PROX_APPLICATION_H */


