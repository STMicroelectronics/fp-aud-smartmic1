/**
  ******************************************************************************
  * @file           : nucleo_f446re_conf.h
  * @brief          : Configuration file
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
#ifndef NUCLEO_F446RE_CONF_H
#define NUCLEO_F446RE_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup BSP
  * @{
  */
  
/** @addtogroup NUCLEO_F446RE
  * @{
  */

/** @defgroup NUCLEO_F446RE_CONFIG Config
  * @{
  */ 
  
/** @defgroup NUCLEO_F446RE_CONFIG_Exported_Constants
  * @{
  */ 
   
/* IRQ priorities */
#define BSP_BUTTON_KEY_IT_PRIORITY          15U

/* I2C1 Frequeny in Hz  */
#define BUS_I2C1_FREQUENCY                  400000U /* Frequency of I2C1 = 400 KHz*/
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
  
#ifdef __cplusplus
}
#endif   
#endif  /* NUCLEO_F446RE_CONF_H */


