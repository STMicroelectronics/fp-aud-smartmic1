/**
  ******************************************************************************
  * @file    prox_application.c
  * @author  SRA
  * 
  * 
  * @brief   proximity sensors related functions.
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
#include "prox_application.h"

/** @defgroup SMARTMIC1
  * @{
  */

/** @defgroup SMARTMIC1_PROX
  * @{
  */


/** @defgroup SMARTMIC1_PROX_Private_Function_Prototypes
  * @{
  */


/**
  * @}
  */

/** @defgroup SMARTMIC1_PROX_Exported_Variables
  * @{
  */
/**
  * @}
  */

/** @defgroup SMARTMIC1_PROX_Private_Variables
  * @{
  */

static void *VL53L0X_0_handler = NULL;
static void *VL53L0X_1_handler = NULL;
static Gesture_DIRSWIPE_1_Data_t gestureDirSwipeData;
extern AudioStatus_t UserAudioStatus;

uint16_t fault_counter = 0;

/** @defgroup SMARTMIC1_PROX_Exported_Function
  * @{
  */

/**
  * @brief  Initialize TOF sensors
  * @param  none
  * @retval None
  */
void TOF_Init(void)
{


  if (BSP_PROX_Init(VL53L0X_0, &VL53L0X_0_handler) != COMPONENT_OK)
  {
    return;
  }

  if (BSP_PROX_Init(VL53L0X_1, &VL53L0X_1_handler) != COMPONENT_OK)
  {
    return;
  }


  UserAudioStatus.GeneralStatus.AvailableModules += ALGO_ACTIVATION_TOF;
  AudioStatus_t *InternalStatus = GetAudioStatusInternal();
  InternalStatus->GeneralStatus.AvailableModules = UserAudioStatus.GeneralStatus.AvailableModules;

  BSP_PROX_Sensor_Enable(VL53L0X_0_handler);
  BSP_PROX_Sensor_Enable(VL53L0X_1_handler);

  /* Initialize directional swipes recognition : swipe detected below 250 mm, no max speed, min duration is 0.5 sec for a swipe and hand must cover both devices */
  tof_gestures_initDIRSWIPE_1(150, 0, 500, true, &gestureDirSwipeData);

  BSP_PROX_Set_DeviceMode(VL53L0X_0_handler, CONTINUOUS);
  BSP_PROX_Set_DeviceMode(VL53L0X_1_handler, CONTINUOUS);

  BSP_PROX_Set_RangingProfile(VL53L0X_0_handler, VL53L0X_PROFILE_HIGH_SPEED);
  BSP_PROX_Set_RangingProfile(VL53L0X_1_handler, VL53L0X_PROFILE_HIGH_SPEED);
}


/**
  * @brief  Periodic TOF processing
  * @param  none
  * @retval None
  */
void PeriodicTOF(void)
{
  static uint8_t readPROX = START_M;

  static uint8_t range_statusL = 0;
  static uint8_t range_statusR = 0;
  uint8_t NewDataReady_0 = 0;
  uint8_t NewDataReady_1 = 0;
  uint8_t complete_Measurement = 0;
  int gesture_code;
  int leakyR = 0;
  int leakyL = 0;
  int32_t leftRange, rightRange;
  uint8_t Volume;
  float used_range = 0.0f;
  uint16_t range_1 = 0;
  uint16_t range = 0;

  /* if proximity sensing is enabled*/
  if ((UserAudioStatus.BeamStatus.Reserved[0] || UserAudioStatus.GeneralStatus.Reserved[0]))
  {
    //read data
    switch (readPROX)
    {
      case START_M:
        BSP_PROX_Start_Measurement(VL53L0X_0_handler);
        BSP_PROX_Start_Measurement(VL53L0X_1_handler);

        readPROX = WAIT_M;
        break;
      case WAIT_M:
        if (NewDataReady_0 == 0)
        {
          BSP_PROX_Get_Measurement_DataReady(VL53L0X_0_handler, &NewDataReady_0);
        }
        if (NewDataReady_1 == 0)
        {
          BSP_PROX_Get_Measurement_DataReady(VL53L0X_1_handler, &NewDataReady_1);
        }

        if (NewDataReady_0 != 0 && NewDataReady_1 != 0)
        {
          readPROX = GET_M;
        }
        break;

      case GET_M:
        BSP_PROX_Get_Range(VL53L0X_0_handler, (uint16_t *)&range);
        BSP_PROX_Get_Range(VL53L0X_1_handler, (uint16_t *)&range_1);

        readPROX = START_M;
        complete_Measurement = 1;
        NewDataReady_0 = 0;
        NewDataReady_1 = 0;
        break;
      default:
        while (1);
    }

    //process data
    if (complete_Measurement)
    {
      /*Gesture Detect on*/
      if (UserAudioStatus.BeamStatus.Reserved[0])
      {

        BSP_PROX_Get_RangeStatus(VL53L0X_0_handler, &range_statusL);
        BSP_PROX_Get_LeakyRange(VL53L0X_0_handler, &leakyL);

        BSP_PROX_Get_RangeStatus(VL53L0X_1_handler, &range_statusR);
        BSP_PROX_Get_LeakyRange(VL53L0X_1_handler, &leakyR);

        leftRange = (range_statusL == 0) ? leakyL : 1200;
        rightRange = (range_statusR == 0) ? leakyR : 1200;

        gesture_code = tof_gestures_detectDIRSWIPE_1(leftRange, rightRange, &gestureDirSwipeData);

        AudioStatus_t *InternalStatus = GetAudioStatusInternal();
        if (gesture_code == GESTURES_SWIPE_LEFT_RIGHT)
        {
          InternalStatus->BeamStatus.Direction = 7;
          UserAudioStatus.BeamStatus.Direction = 7;
        }
        else if (gesture_code == GESTURES_SWIPE_RIGHT_LEFT)
        {
          InternalStatus->BeamStatus.Direction = 3;
          UserAudioStatus.BeamStatus.Direction = 3;
        }
        BeamDirectionSetup(UserAudioStatus.BeamStatus.Direction);
      }

      /*Volume Gesture on*/
      if (UserAudioStatus.GeneralStatus.Reserved[0])
      {
        AudioStatus_t *InternalStatus = GetAudioStatusInternal();
        int temp = abs((int)range - (int)range_1);

        if (temp < 100)
        {
          used_range = (float)range;
          if (used_range > 500.0f)
          {
            used_range = 500.0f;
          }
          Volume = (uint8_t)(127.0f * used_range / 500.0f);

          UserAudioStatus.GeneralStatus.Reserved[1] = (uint32_t)(Volume);
          InternalStatus->GeneralStatus.Reserved[1] = UserAudioStatus.GeneralStatus.Reserved[1];
          UserAudioStatus.GeneralStatus.Volume = UserAudioStatus.GeneralStatus.Reserved[1];
          InternalStatus->GeneralStatus.Volume = UserAudioStatus.GeneralStatus.Reserved[1];

          BSP_AUDIO_IN_SetVolume(BSP_AUDIO_IN_INSTANCE, (uint32_t)Volume);
        }

      }
      complete_Measurement = 0;
    }
  }
}




/** @defgroup SMARTMIC1_PROX_Private_Function_Prototypes
  * @{
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

/**
  * @}
  */


