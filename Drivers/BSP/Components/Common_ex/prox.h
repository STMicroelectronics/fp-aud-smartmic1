/**
 ******************************************************************************
 * @file    prox.h
 * @author  Central Lab
 * @version V1.0.0
 * @date    26-May-2017
 * @brief   This header file contains the functions prototypes for the
 *          range driver
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
#ifndef __PROX_H
#define __PROX_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "sensor.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup COMMON COMMON
 * @{
 */

/** @addtogroup PROX PROX
 * @{
 */

/** @addtogroup PROX_Public_Types PROX Public types
 * @{
 */

/**
 * @brief  PROX driver structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Init           ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit         ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI     ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI   ) ( DrvContextTypeDef* );
  
  DrvStatusTypeDef ( *Get_Range      ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR        ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR        ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Set_RangingProfile  ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Get_RangingProfile  ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Set_Device_Mode  ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Get_Device_Mode  ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Get_RangeStatus  ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Get_LeakyRange  ) ( DrvContextTypeDef*, int* );
} PROX_Drv_t;




/**
 * @brief  range data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */ 
  void *pExtData;       /* Other data. */
} PROX_Data_t;

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

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __PROX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
