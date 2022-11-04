/**
******************************************************************************
* @file    BlueCoin_PROX.c
* @author  SRA - Central Labs
* @version v2.2.3
* @date    10-Feb-2022
* @brief   This file provides the range sensor driver
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


/* Includes ------------------------------------------------------------------*/
#include "BlueCoin_prox.h"
#include "BlueCoin.h"


/** @addtogroup BSP
 * @{
 */

/** @addtogroup BlueCoin
 * @{
 */

/** @addtogroup BlueCoin_PROXIMITY
* @brief This file provides a set of firmware functions to manage MEMS range Sensor
 * @{
 */

#include "../../STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"
#include <string.h>

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1
#define VL53L0X_OsDelay(...) HAL_Delay(2)


#ifndef HAL_I2C_MODULE_ENABLED
#warning "HAL I2C module must be enable "
#endif

/* when not customized by application define dummy one */
#ifndef VL53L0X_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_GetI2cBus(...) (void)0
#endif

#ifndef VL53L0X_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_PutI2cBus(...) (void)0
#endif

#ifndef VL53L0X_OsDelay
#   define  VL53L0X_OsDelay(...) (void)0
#endif




/** @defgroup BlueCoin_PROXIMITY_Private_Variables 
 * @{
 */

static DrvContextTypeDef PROX_SensorHandle[ PROX_SENSORS_MAX_NUM ];
static PROX_Data_t PROX_Data[ PROX_SENSORS_MAX_NUM ];
static VL53L0X_Data_t PROX_X_0_Data;
static VL53L0X_Data_t PROX_X_1_Data;
static uint8_t _I2CBuffer[64];
static VL53L0X_MODE_t old_mode[PROX_SENSORS_MAX_NUM];

/**
 * @}
 */

/** @defgroup BlueCoin_PROXIMITY_Private_Function_Prototypes 
 * @{
 */

static DrvStatusTypeDef BSP_VL53L0X_PROX_Init(PROX_ID_t id, void **handle );
static DrvStatusTypeDef VL53L0X_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef VL53L0X_Set_RangingProfile( DrvContextTypeDef *handle, uint8_t value );
static DrvStatusTypeDef VL53L0X_Set_RangingProfile( DrvContextTypeDef *handle, uint8_t value );
static DrvStatusTypeDef VL53L0X_Set_Device_Mode( DrvContextTypeDef *handle, uint8_t value );
static DrvStatusTypeDef VL53L0X_Get_RangeStatus( DrvContextTypeDef *handle, uint8_t *value );
static DrvStatusTypeDef VL53L0X_Get_LeakyRange( DrvContextTypeDef *handle, int *value );
static void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange);
static void BSP_VL53L0X_SetAddress( DrvContextTypeDef *handle, uint8_t newAddress);


/**
 * @}
 */

/** @defgroup BlueCoin_PROXIMITY_Imported_Function_Prototypes 
 * @{
 */

/* Sensor IO functions */
//extern DrvStatusTypeDef Sensor_IO_Init( void );
//extern DrvStatusTypeDef Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
//extern DrvStatusTypeDef Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
extern I2C_HandleTypeDef    I2C_ONBOARD_SENSORS_Handle;
extern volatile uint8_t IT_I2C_error_af;

/**
 * @}
 */

/** @defgroup BlueCoin_PROXIMITY_Public_Functions  
 * @{
 */

/**
 * @brief Initialize a range sensor
 * @param id the range sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Init( PROX_ID_t id, void **handle )
{
  *handle = NULL;
  
  switch(id)
  {
    case PROX_SENSORS_AUTO:
    default:
    {
      if( BSP_VL53L0X_PROX_Init(VL53L0X_0, handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case VL53L0X_0:
    {
      if( BSP_VL53L0X_PROX_Init(VL53L0X_0, handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case VL53L0X_1:
    {
      if( BSP_VL53L0X_PROX_Init(VL53L0X_1, handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      
      break;
    }
  }

  return COMPONENT_OK;
}

static DrvStatusTypeDef BSP_VL53L0X_PROX_Init(PROX_ID_t id, void **handle )
{
  
  if ( BSP_I2C1_Init() != BSP_ERROR_NONE )
  {
    return COMPONENT_ERROR;
  }
  
  switch(id)
  {
    case VL53L0X_1:
      // the U5 PROX must be initialize before U6 
      if(PROX_SensorHandle[ VL53L0X_1 ].isInitialized == 1 || (PROX_SensorHandle[ VL53L0X_0 ].isInitialized != 1))
      {
        return COMPONENT_ERROR;
      }
      
      //switch on the second PROX (U6)
      BSP_PROX_Pin_init();
      BSP_PROX_Pin_On();
      HAL_Delay(1);

      /* Setup sensor handle. */
      PROX_SensorHandle[ VL53L0X_1 ].who_am_i      = 0;
      PROX_SensorHandle[ VL53L0X_1 ].ifType        = 0;
      PROX_SensorHandle[ VL53L0X_1 ].address       = VL53L0X_ADDRESS_DEFAULT;
      PROX_SensorHandle[ VL53L0X_1 ].instance      = VL53L0X_1;
      PROX_SensorHandle[ VL53L0X_1 ].isInitialized = 0;
      PROX_SensorHandle[ VL53L0X_1 ].isEnabled     = 1;
      PROX_SensorHandle[ VL53L0X_1 ].isCombo       = 0;
      PROX_SensorHandle[ VL53L0X_1 ].pData         = ( void * )&PROX_Data[ VL53L0X_1 ];
      PROX_SensorHandle[ VL53L0X_1 ].pVTable       = NULL;
      PROX_SensorHandle[ VL53L0X_1 ].pExtVTable    = 0;
      
      PROX_Data[ VL53L0X_1 ].pComponentData = ( void * )&PROX_X_1_Data;
      PROX_Data[ VL53L0X_1 ].pExtData       = 0;
      
      *handle = (void *)&PROX_SensorHandle[ VL53L0X_1 ];
      
      break;
      
    case VL53L0X_0:
      if(PROX_SensorHandle[ VL53L0X_0 ].isInitialized == 1)
      {
        /* We have reached the max num of instance for this component */
        return COMPONENT_ERROR;
      }
      /* Setup sensor handle. */
      PROX_SensorHandle[ VL53L0X_0 ].who_am_i      = 0;
      PROX_SensorHandle[ VL53L0X_0 ].ifType        = 0;
      PROX_SensorHandle[ VL53L0X_0 ].address       = VL53L0X_ADDRESS_DEFAULT;
      PROX_SensorHandle[ VL53L0X_0 ].instance      = VL53L0X_0;
      PROX_SensorHandle[ VL53L0X_0 ].isInitialized = 0;
      PROX_SensorHandle[ VL53L0X_0 ].isEnabled     = 1;
      PROX_SensorHandle[ VL53L0X_0 ].isCombo       = 0;
      PROX_SensorHandle[ VL53L0X_0 ].pData         = ( void * )&PROX_Data[ VL53L0X_0 ];
      PROX_SensorHandle[ VL53L0X_0 ].pVTable       = NULL;
      PROX_SensorHandle[ VL53L0X_0 ].pExtVTable    = 0;
      
      PROX_Data[ VL53L0X_0 ].pComponentData = ( void * )&PROX_X_0_Data;
      PROX_Data[ VL53L0X_0 ].pExtData       = 0;
      
      *handle = (void *)&PROX_SensorHandle[ VL53L0X_0 ];
   
      break;
      
    default:
      return COMPONENT_ERROR;
  }
  
  if ( VL53L0X_Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

static DrvStatusTypeDef VL53L0X_Init( DrvContextTypeDef *handle )
{
    uint8_t Id=0x00;
    VL53L0X_Error status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    VL53L0X_Dev_t *pDev;

    pDev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->dev);
    pDev->Present = 0;     
    pDev->Id= (uint8_t)((DrvContextTypeDef *) handle)->instance;
    
    
    status=VL53L0X_RdByte(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID,  &Id);
    
    if (IT_I2C_error_af == 1)
    {
      if(pDev->Id== VL53L0X_0)
      {
        BSP_VL53L0X_SetAddress(handle, VL53L0X_ADDRESS_U5);
      }
      else
      {
        BSP_VL53L0X_SetAddress(handle, VL53L0X_ADDRESS_U6);
      }
      
      Sensor_IO_Error();
           
      IT_I2C_error_af=0;
      /* Set I2C standard mode (400 KHz) before doing the first register access */
      status=VL53L0X_WrByte(pDev, 0x88, 0x00); 
    }
    else
    {
      status=VL53L0X_WrByte(pDev, 0x88, 0x00);
      
      status=VL53L0X_RdByte(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID,  &Id);
      if(status == VL53L0X_ERROR_NONE)
      {
        
        if(pDev->Id== VL53L0X_0)
        {
          status = VL53L0X_SetDeviceAddress(pDev, VL53L0X_ADDRESS_U5);
        }
        else
        {
          status = VL53L0X_SetDeviceAddress(pDev, VL53L0X_ADDRESS_U6);
        }
        
        if( status != VL53L0X_ERROR_NONE )
        {
          return COMPONENT_ERROR;
        }
      }
      
      if(pDev->Id== VL53L0X_0)
      {
        BSP_VL53L0X_SetAddress(handle, VL53L0X_ADDRESS_U5);
      }
      else
      {
        BSP_VL53L0X_SetAddress(handle, VL53L0X_ADDRESS_U6);
      }
      
    }
    
    HAL_Delay(3);
    
    status = VL53L0X_DataInit(pDev);
    if( status != VL53L0X_ERROR_NONE )
    {
      return COMPONENT_ERROR;
    }
    pDev->Present = 1;
    
    if( VL53L0X_StaticInit(pDev) != VL53L0X_ERROR_NONE )
    {
      return COMPONENT_ERROR;
    }
    
    if( VL53L0X_PerformRefCalibration(pDev, &VhvSettings, &PhaseCal)  != VL53L0X_ERROR_NONE )
    {
      return COMPONENT_ERROR;
    }
   
    if( VL53L0X_PerformRefSpadManagement(pDev, &refSpadCount, &isApertureSpads)  != VL53L0X_ERROR_NONE )
    {
      return COMPONENT_ERROR;
    }
    
    pDev->LeakyFirst=1;
    handle->isInitialized = 1;
    
    return COMPONENT_OK;
}

/**
 * @brief Set new address on handler level
 * @param handle the device handle
 * @param newAddress new address to be set
 */
static void BSP_VL53L0X_SetAddress( DrvContextTypeDef *handle, uint8_t newAddress)
{
    ((DrvContextTypeDef *)(handle))->address = newAddress;
}

/**
 * @brief Deinitialize a range sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_DeInit( void **handle )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  
  VL53L0X_Dev_t *dev;
  
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  dev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) (ctx)->pData)->pComponentData)->dev);

  if(ctx ->isInitialized == 1)
  {
    VL53L0X_StopMeasurement(dev);
    
    VL53L0X_ResetDevice(dev);
    
    VL53L0X_SetPowerMode(dev, VL53L0X_POWERMODE_IDLE_LEVEL1);
    
    if(ctx->instance == VL53L0X_1)
    {
      //switch off the second PROX (U6)
      BSP_PROX_Pin_Off();

      HAL_Delay(5);
     }
    ctx->isInitialized = 0;

  }
  else
  {
    return COMPONENT_ERROR;
  }
  
  memset(ctx, 0, sizeof(DrvContextTypeDef));
  
  *handle = NULL;
  
  return COMPONENT_OK;
}



/**
 * @brief Enable range sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Sensor_Enable( void *handle )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  uint8_t status;
  

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  BSP_PROX_IsEnabled(ctx, &status);
  if(status)
  {
    return COMPONENT_ERROR;
  }
  
  ctx->isEnabled=1;
  BSP_PROX_Set_DeviceMode(handle, old_mode[ctx->instance]);

  return COMPONENT_OK;
}



/**
 * @brief Disable range sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Sensor_Disable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  uint8_t status;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  BSP_PROX_IsEnabled(ctx, &status);
  if(!status)
  {
    return COMPONENT_ERROR;
  }
  
  ctx->isEnabled=0;
  BSP_PROX_Set_DeviceMode(handle, SINGLE);

  
  return COMPONENT_OK;
}


/**
 * @brief Check if the range sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_IsInitialized( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isInitialized;
  
  return COMPONENT_OK;
}


/**
 * @brief Check if the range sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_IsEnabled( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isEnabled;
  
  return COMPONENT_OK;
}


/**
 * @brief Check if the range sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_IsCombo( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isCombo;
  
  return COMPONENT_OK;
}


/**
 * @brief Get the range sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Get_Instance( void *handle, uint8_t *instance )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( instance == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *instance = ctx->instance;
  
  return COMPONENT_OK;
}

/**
 * @brief Start ranging measurement
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Start_Measurement( void *handle)
{
  VL53L0X_Dev_t *pDev;
  uint8_t status = 0;
  
  
  BSP_PROX_IsEnabled(handle, &status);
  if(!status)
  {
    return COMPONENT_ERROR;
  }
  
  pDev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->dev);
  VL53L0X_StartMeasurement(pDev);
  pDev->Ready=0;

  return COMPONENT_OK;
}

/**
 * @brief Get status flag on measurement ready
 * @param handle the device handle
 * @param pDataready pointer to the return variable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Get_Measurement_DataReady( void *handle,uint8_t *pDataready)
{  
 VL53L0X_Dev_t *pDev;
 uint8_t status = 0;
 
 
 BSP_PROX_IsEnabled(handle, &status);
 if(!status)
 {
   return COMPONENT_ERROR;
 }
 
 pDev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->dev);
 status= VL53L0X_GetMeasurementDataReady(pDev, pDataready);
 
 if(status != VL53L0X_ERROR_NONE)
   return COMPONENT_ERROR;
 
 return COMPONENT_OK;
}

/**
 * @brief Get the range value
 * @param handle the device handle
 * @param range pointer where the value is written [mm]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Get_Range( void *handle, uint16_t *range )
{
  VL53L0X_Dev_t *pDev;
  uint8_t status = 0;
  VL53L0X_RangingMeasurementData_t RangingMeasurementData;
  
  
  BSP_PROX_IsEnabled(handle, &status);
  if(!status)
  {
    return COMPONENT_ERROR;
  }
  pDev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->dev);
  
  /* Clear Interrupt */
  status = VL53L0X_ClearInterruptMask(pDev, 0);
  if( status )
  {
    return COMPONENT_ERROR;
  }
  /* Otherwise, get new sample data and store */
  status = VL53L0X_GetRangingMeasurementData(pDev, &RangingMeasurementData);
  if( status )
  {
    return COMPONENT_ERROR;
  }
  
  Sensor_SetNewRange(pDev,&RangingMeasurementData);
  pDev->Ready=1;
  
  (*range) = RangingMeasurementData.RangeMilliMeter;
  
  return COMPONENT_OK;
  
}

/**
 * @brief Get the current ranging profile 
 * @param handle the device handle
 * @param status pointer to the destination variable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Get_RangingProfile( void *handle, uint8_t *status )
{
  
  VL53L0X_Data_t *pData = (VL53L0X_Data_t *) ((DrvContextTypeDef *) handle) ->pData;
  
  if(pData == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  (*status)=pData->profile;
  
  
  return COMPONENT_OK;

}

/**
 * @brief Set the current ranging profile 
 * @param handle the device handle
 * @param value should assume one of the following values
 *            @arg  VL53L0X_PROFILE_HIGH_SPEED
 *            @arg  VL53L0X_PROFILE_HIGH_ACCUR
 *            @arg  VL53L0X_PROFILE_LONG_RANGE
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Set_RangingProfile( void *handle, uint8_t value )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }


  if(value == VL53L0X_PROFILE_HIGH_SPEED || value == VL53L0X_PROFILE_HIGH_ACCUR || value == VL53L0X_PROFILE_LONG_RANGE )
  {
    if ( VL53L0X_Set_RangingProfile( ctx, value ) != COMPONENT_ERROR )
    {
      return COMPONENT_OK;
    }
  }
  
  return COMPONENT_ERROR;
}


static DrvStatusTypeDef VL53L0X_Set_RangingProfile( DrvContextTypeDef *handle, uint8_t value ){
  
  VL53L0X_Error status;
  
  VL53L0X_Dev_t *dev;
  uint8_t *p;
  
  uint8_t VhvSettings;
  uint8_t PhaseCal;

  FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
  FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
  uint32_t timingBudget = 33000;
  uint8_t preRangeVcselPeriod = 14;
  uint8_t finalRangeVcselPeriod = 10;
  
  dev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->dev);
  p = (uint8_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->profile);
  *p=value;
  
  status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
  if( status )
  {
    return COMPONENT_ERROR;
  }
  
  status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
  if( status )
  {
    return COMPONENT_ERROR;
  }
  
  switch(value) 
  {
    case VL53L0X_PROFILE_LONG_RANGE:
      signalLimit = (FixPoint1616_t)(0.1*65536);
      sigmaLimit = (FixPoint1616_t)(60*65536);
      timingBudget = 33000;
      preRangeVcselPeriod = 18;
      finalRangeVcselPeriod = 14;
      break;
    case VL53L0X_PROFILE_HIGH_ACCUR:
      signalLimit = (FixPoint1616_t)(0.25*65536);
      sigmaLimit = (FixPoint1616_t)(18*65536);
      timingBudget = 200000;
      preRangeVcselPeriod = 14;
      finalRangeVcselPeriod = 10;
      break;
    case VL53L0X_PROFILE_HIGH_SPEED:
      signalLimit = (FixPoint1616_t)(0.25*65536);
      sigmaLimit = (FixPoint1616_t)(32*65536);
      timingBudget = 20000;
      preRangeVcselPeriod = 14;
      finalRangeVcselPeriod = 10;
      break;
    default:
      return COMPONENT_ERROR;
  }
  
  status = VL53L0X_SetLimitCheckValue(dev,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
  if( status )
  {
    return COMPONENT_ERROR;
  }
  
  status = VL53L0X_SetLimitCheckValue(dev,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
  if( status )
  {
    return COMPONENT_ERROR;
  }
  
  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,  timingBudget);
  if( status )
  {
    return COMPONENT_ERROR;
  }
  
  status = VL53L0X_SetVcselPulsePeriod(dev,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
  if( status )
  {
    return COMPONENT_ERROR;
  }
  
  status = VL53L0X_SetVcselPulsePeriod(dev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
  if( status )
  {
    return COMPONENT_ERROR;
  }
  
  status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
  if( status )
  {
    return COMPONENT_ERROR;
  }

  dev->LeakyFirst=1;
  
  return COMPONENT_OK;
}

/**
 * @brief Get the current device mode
 * @param handle the device handle
 * @param status pointer to the destination variable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Get_Device_Mode( void *handle, uint8_t *status )
{
    VL53L0X_Dev_t *pDev;
    pDev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->dev);

    
    if( VL53L0X_GetDeviceMode(pDev, (uint8_t *) status) )
    {
      return COMPONENT_ERROR;
    }
    
    return COMPONENT_OK;
   
}

/**
 * @brief Set the current device mode
 * @param handle the device handle
 * @param value should assume one of the following values
 *            @arg  SINGLE
 *            @arg  CONTINUOUS
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROX_Set_DeviceMode( void *handle, VL53L0X_MODE_t value )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *) handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if (  value > TIMED )
  {
    return COMPONENT_ERROR;
  }

  if ( VL53L0X_Set_Device_Mode( ctx, value ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  old_mode[ctx->instance]=value;
  
  return COMPONENT_OK;
}

static DrvStatusTypeDef VL53L0X_Set_Device_Mode( DrvContextTypeDef *handle, uint8_t value )
{

  VL53L0X_Dev_t *pDev;
  VL53L0X_Error status;
  pDev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->dev);
  
  if( pDev->Present){
    switch( (VL53L0X_MODE_t) value){
    case SINGLE :
      status = VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
      if( status )
      {
        return COMPONENT_ERROR;
      }
      break;
    case CONTINUOUS:
      status = VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
      if( status )
      {
        return COMPONENT_ERROR;
      }
      break;
    case TIMED:
      return COMPONENT_NOT_IMPLEMENTED; /*interrupt pin is needed*/
 
    default:
      status = VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
      if( status )
      {
        return COMPONENT_ERROR;
      } 
    }
  }
  else
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

DrvStatusTypeDef BSP_PROX_Get_RangeStatus( void *handle, uint8_t *value )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)  handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  
  if ( value == NULL )
  {
    return COMPONENT_ERROR;
  }

  if (VL53L0X_Get_RangeStatus( ctx, value ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
  
}

static DrvStatusTypeDef VL53L0X_Get_RangeStatus( DrvContextTypeDef *handle, uint8_t *value ){
  
  VL53L0X_Dev_t *pDev;
  pDev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->dev);
  
  (*value) = pDev->RangeStatus;
  
  return COMPONENT_OK;
}

DrvStatusTypeDef BSP_PROX_Get_LeakyRange( void *handle, int *value )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)  handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
    
  if ( value == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( VL53L0X_Get_LeakyRange( ctx, value ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
  
}

static DrvStatusTypeDef VL53L0X_Get_LeakyRange( DrvContextTypeDef *handle, int *value ){
  
  VL53L0X_Dev_t *pDev;
  pDev = (VL53L0X_Dev_t *)  &(((VL53L0X_Data_t *) ((PROX_Data_t *) ((DrvContextTypeDef *) handle)->pData)->pComponentData)->dev);
    
  (*value) = pDev->LeakyRange;
  return COMPONENT_OK;
}


/* Store new ranging data into the device structure, apply leaky integrator if needed */
static void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange)
{
  int LeakyFactorFix8 = (int)( 0.0 *256);

    if( pRange->RangeStatus == 0 )
    {
    	pDev->RangeStatus = 0;
        if( pDev->LeakyFirst )
        {
            pDev->LeakyFirst = 0;
            pDev->LeakyRange = pRange->RangeMilliMeter;
        }
        else
        {
            pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
        }
    }
    else
    {
    	pDev->RangeStatus = pRange->RangeStatus;
    	pDev->LeakyFirst = 1;
    }
}


VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
  
  if (count > sizeof(_I2CBuffer) - 1)
  {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  
  if( BSP_I2C1_WriteReg( PROX_SensorHandle[Dev->Id].address,  (uint16_t)index, pdata, (uint16_t) count ) == COMPONENT_OK)
  {
    return VL53L0X_ERROR_NONE;
  }
  
  return VL53L0X_ERROR_CONTROL_INTERFACE;

}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
  
  if (count > sizeof(_I2CBuffer) - 1)
  {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  
  if( BSP_I2C1_ReadReg( PROX_SensorHandle[Dev->Id].address,  (uint16_t)index, pdata, (uint16_t) count ) == COMPONENT_OK)
  {
    return VL53L0X_ERROR_NONE;
  }
  
  return VL53L0X_ERROR_CONTROL_INTERFACE;

}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{

  if( BSP_I2C1_WriteReg( PROX_SensorHandle[Dev->Id].address,  (uint16_t)index, &data,1 ) == COMPONENT_OK)
  {
    return VL53L0X_ERROR_NONE;
  }
  
  return VL53L0X_ERROR_CONTROL_INTERFACE;

}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
  
  _I2CBuffer[0] = data >> 8;
  _I2CBuffer[1] = data & 0x00FF;
  
  if( BSP_I2C1_WriteReg( PROX_SensorHandle[Dev->Id].address,  (uint16_t)index, _I2CBuffer, 2 ) == COMPONENT_OK)
  {    
    return VL53L0X_ERROR_NONE;
  }
 
  return VL53L0X_ERROR_CONTROL_INTERFACE;

}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
  
  _I2CBuffer[0] = (data >> 24) & 0xFF;
  _I2CBuffer[1] = (data >> 16) & 0xFF;
  _I2CBuffer[2] = (data >> 8)  & 0xFF;
  _I2CBuffer[3] = (data >> 0 ) & 0xFF;
  
  if( BSP_I2C1_WriteReg( PROX_SensorHandle[Dev->Id].address,  (uint16_t)index, _I2CBuffer, 4 ) == COMPONENT_OK)
  {    
    return VL53L0X_ERROR_NONE;
  }
  
  return VL53L0X_ERROR_CONTROL_INTERFACE;

}


VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint8_t data;
  
  Status = VL53L0X_RdByte(Dev, index, &data);
  if (Status)
  {
    goto done;
  }
  data = (data & AndData) | OrData;
  Status = VL53L0X_WrByte(Dev, index, data);
done:
  return Status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
  
  if( BSP_I2C1_ReadReg( PROX_SensorHandle[Dev->Id].address, (uint16_t)index, data,1 ) == COMPONENT_OK)
  {
    return VL53L0X_ERROR_NONE;
  }
  
  return VL53L0X_ERROR_CONTROL_INTERFACE;

}


VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
  if( BSP_I2C1_ReadReg( PROX_SensorHandle[Dev->Id].address, (uint16_t)index, _I2CBuffer, 2 ) == COMPONENT_OK)
  {
     *data = ((uint16_t)_I2CBuffer[0] << 8) + (uint16_t)_I2CBuffer[1];
  
    return VL53L0X_ERROR_NONE;
  }
  
  return VL53L0X_ERROR_CONTROL_INTERFACE;
  
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
  
  if( BSP_I2C1_ReadReg( PROX_SensorHandle[Dev->Id].address, (uint16_t)index, _I2CBuffer, 4 ) == COMPONENT_OK)
  {
    *data = ((uint32_t)_I2CBuffer[0] << 24) + ((uint32_t)_I2CBuffer[1] << 16) + ((uint32_t)_I2CBuffer[2] << 8) +
          (uint32_t)_I2CBuffer[3];
  
    return VL53L0X_ERROR_NONE;
  }

  return VL53L0X_ERROR_CONTROL_INTERFACE;

}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  
  // do nothing
  VL53L0X_OsDelay();

  return status;
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
 
 /**
 * @}
 */

