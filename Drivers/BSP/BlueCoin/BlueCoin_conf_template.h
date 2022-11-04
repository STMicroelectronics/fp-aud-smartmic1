/**
******************************************************************************
* @file    BlueCoin_conf_template.h
* @author  SRA - Central Labs
* @version v2.2.3
* @date    10-Feb-2022
* @brief   This file contains definitions for the components bus interfaces
*          This file should be copied to the application folder and renamed
*          to BlueCoin_conf.h.
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
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLUECOIN_CONF_H__
#define __BLUECOIN_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "BlueCoin_bus.h"
#include "BlueCoin_errno.h"
  
/*Uncomment this define if you want to configure and start acquisition 
independently from USB functionalities*/
#define DISABLE_USB_DRIVEN_ACQUISITION
  
/* The N_MS value defines the number of millisecond to be processed at each AudioProcess call,
that must be consistent with the N_MS_PER_INTERRUPT defined in the audio driver
(cca02m1_audio.h).
The default value of the N_MS_PER_INTERRUPT directive in the driver is set to 1, 
for backward compatibility: leaving this values as it is allows to avoid any 
modification in the application layer developed with the older versions of the driver */

#define N_MS (N_MS_PER_INTERRUPT)  
  
#define AUDIO_CHANNELS 				2
#define AUDIO_SAMPLING_FREQUENCY 		16000  
#define BSP_AUDIO_IN_INSTANCE                   0U   /* Define the audio peripheral used: 0U = I2S */  
#define BSP_AUDIO_OUT_INSTANCE                  0U   /* Define the audio peripheral used: 0U = SAI */  

#define AUDIO_IN_BUFFER_SIZE            DEFAULT_AUDIO_IN_BUFFER_SIZE
#define AUDIO_VOLUME_INPUT                      64U
#define AUDIO_VOLUME_OUTPUT                     50U
#define BSP_AUDIO_IN_IT_PRIORITY                6U
  
#if (AUDIO_SAMPLING_FREQUENCY == 8000)
#define MAX_DECIMATION_FACTOR 160
#else
#define MAX_DECIMATION_FACTOR 128
#endif

#define USE_MOTION_SENSOR_LSM6DSM_0             0U
#define USE_MOTION_SENSOR_LSM303AGR_ACC_0       0U
#define USE_MOTION_SENSOR_LSM303AGR_MAG_0       0U
#define USE_ENV_SENSOR_LPS22HB_0                0U

#define BSP_LSM6DSM_INT1_GPIO_PORT           GPIOA
#define BSP_LSM6DSM_INT1_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define BSP_LSM6DSM_INT1_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define BSP_LSM6DSM_INT1                 GPIO_PIN_0
#define BSP_LSM6DSM_INT1_EXTI_IRQn           EXTI0_IRQn 

#define BSP_LSM6DSM_INT2_GPIO_PORT           GPIOC
#define BSP_LSM6DSM_INT2_GPIO_CLK_ENABLE()   __GPIOC_CLK_ENABLE()
#define BSP_LSM6DSM_INT2_GPIO_CLK_DISABLE()  __GPIOC_CLK_DISABLE()
#define BSP_LSM6DSM_INT2                 GPIO_PIN_4
#define BSP_LSM6DSM_INT2_EXTI_IRQn           EXTI4_IRQn 

#define BSP_LSM303AGR_INT_GPIO_PORT           GPIOB
#define BSP_LSM303AGR_INT_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define BSP_LSM303AGR_INT_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define BSP_LSM303AGR_INT                 GPIO_PIN_1
#define BSP_LSM303AGR_INT_EXTI_IRQn           EXTI1_IRQn 

#define BSP_LPS22HB_INT_GPIO_PORT           GPIOB
#define BSP_LPS22HB_INT_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define BSP_LPS22HB_INT_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define BSP_LPS22HB_INT                 GPIO_PIN_3
#define BSP_LPS22HB_INT_EXTI_IRQn           EXTI1_IRQn 


#ifdef __cplusplus
}
#endif

#endif /* __BLUECOIN_CONF_H__*/
