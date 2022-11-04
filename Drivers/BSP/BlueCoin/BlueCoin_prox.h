/**
******************************************************************************
* @file    BlueCoin_PROX.h
* @author  SRA - Central Labs
* @version v2.2.3
* @date    10-Feb-2022
* @brief   This file contains definitions for BlueCoin_PROX.c driver.
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
#ifndef __BLUECOIN_PROX_H
#define __BLUECOIN_PROX_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "BlueCoin.h"
#include "prox.h"

/* Include PROX sensor component drivers. */
#include "../Components/vl53l0x/vl53l0x_api.h"
#include "../Components/vl53l0x/vl53l0x_platform.h"
  
  
#include <math.h>
/** @addtogroup BSP
 * @{
 */


/** @addtogroup BlueCoin_PROXIMITY
 * @{
 */

/** @addtogroup BlueCoin_PROXIMITY_Public_Types  
  * @{
  */

typedef enum
{
  PROX_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  VL53L0X_0,                    /* U5 PROX */
  VL53L0X_1                             /* U6 PROX */
} PROX_ID_t;


/** @addtogroup BlueCoin_PROXIMITY_VL53L0X_I2C_Addresses
 * @{
 */
#define VL53L0X_ADDRESS_DEFAULT         (uint8_t)  0x52
#define VL53L0X_ADDRESS_U6              (uint8_t) (VL53L0X_ADDRESS_DEFAULT+4)  /**< VL53L0X I2C Address U6  PROX */
#define VL53L0X_ADDRESS_U5              (uint8_t) (VL53L0X_ADDRESS_DEFAULT+2)  /**< VL53L0X I2C Address U5 PROX */
   
   
#define  VL53L0X_PROFILE_HIGH_SPEED     0
#define  VL53L0X_PROFILE_HIGH_ACCUR     1       
#define  VL53L0X_PROFILE_LONG_RANGE     2

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup BlueCoin_PROXIMITY_Public_Types 
 * @{
 */





typedef enum
{
  SINGLE,
  CONTINUOUS, 
  TIMED
} VL53L0X_MODE_t;



/**
 * @brief VL53L0X range specific data internal structure definition
 */
typedef struct
{
  uint8_t isInitialized;
  uint8_t profile;
  VL53L0X_Dev_t dev;
  float Last_ODR;

} VL53L0X_Data_t;


/**
 * @}
 */

/** @addtogroup BlueCoin_PROXIMITY_Public_Variables
 * @{
 */

extern PROX_Drv_t Vl53l0X_Drv;

/**
 * @}
 */

/** @addtogroup BlueCoin_PROXIMITY_Public_Defines  
  * @{
  */

#define PROX_SENSORS_MAX_NUM 2

/**
 * @}
 */

/** @addtogroup BlueCoin_PROXIMITY_Public_Functions 
 * @{
 */

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_PROX_Init( PROX_ID_t id, void **handle );
DrvStatusTypeDef BSP_PROX_DeInit( void **handle );
DrvStatusTypeDef BSP_PROX_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_PROX_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_PROX_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_PROX_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_PROX_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_PROX_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_PROX_Get_RangingProfile( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_PROX_Set_RangingProfile( void *handle, uint8_t value );
DrvStatusTypeDef BSP_PROX_Get_Device_Mode( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_PROX_Set_DeviceMode( void *handle, VL53L0X_MODE_t value );
DrvStatusTypeDef BSP_PROX_Get_Measurement_DataReady( void *handle, uint8_t *pDataready);
DrvStatusTypeDef BSP_PROX_Start_Measurement( void *handle);
DrvStatusTypeDef BSP_PROX_Get_Range( void *handle, uint16_t *range );
DrvStatusTypeDef BSP_PROX_Get_RangeStatus( void *handle, uint8_t *value );
DrvStatusTypeDef BSP_PROX_Get_LeakyRange( void *handle, int *value );



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

#endif /* __BLUECOIN_PROX_H */
