/**
******************************************************************************
* @file    BlueCoin.h
* @author  SRA - Central Labs
* @version v2.2.3
* @date    10-Feb-2022
* @brief   This file contains definitions for BlueCoin.c file.
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
#ifndef __BLUECOIN_H
#define __BLUECOIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "BlueCoin_conf.h"
#include "component.h"


/** @addtogroup BSP
  * @{
  */

/** @addtogroup BlueCoin
  * @{
  */

/** @addtogroup BlueCoin_COMMON BlueCoin_COMMON
  * @{
  */

/** @defgroup BlueCoin_COMMON_Exported_Types BlueCoin_COMMON_Exported_Types
  * @{
  */
typedef enum
{
  LED1 = 0,
  LED_0 = LED1,
  LED2 = 1,
  LED_45 = LED2,
  LED3 = 2,
  LED_90 = LED3,
  LED4 = 3,
  LED_135 = LED4,
  LED5 = 4,
  LED_180 = LED5,
  LED6 = 5,
  LED_225 = LED6,
  LED7 = 6,
  LED_270 = LED7,
  LED8 = 7,
  LED_315 = LED8
} Led_TypeDef;


typedef enum
{
  BUTTON_1 = 0,
  BUTTON_2 = 1
} Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

typedef enum
{
  CLK_DISABLE  = 0,
  CLK_ENABLE = 1,
} Mic234Clk_Status_TypeDef;

typedef enum
{
  CHRG_STATUS_DISCHARGING = 0,
  CHRG_STATUS_EOC, /* End of Charging */
  CHRG_STATUS_CHARGING, /* Charging phase */
  CHRG_STATUS_OVER_CHRG, /* Over-charge fault */
  CHRG_STATUS_TIMEOUT, /* Charging timeout */
  CHRG_STATUS_BEL_VPPRE, /* Battery voltage below VPRE after the fast-charge starts */
  CHRG_STATUS_THERMAL_WARNING, /* Charging thermal limitation (thermal warning) */
  CHRG_STATUS_NTC_WARNING /* Charging thermal limitation (thermal warning) */
} ChrgStatus_t;

/**
  * @}
  */

  
#define MIC234_CLK_EN_PIN                   GPIO_PIN_15
#define MIC234_CLK_EN_GPIO_PORT             GPIOA
#define MIC234_CLK_EN_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()
#define MIC234_CLK_EN_GPIO_CLK_DISABLE()    __GPIOA_CLK_DISABLE()


#define STBC03J_CHG_FREQ_TH              (0.5f)
#define STBC03J_CHG_EOC                  (4.1f)
#define STBC03J_CHG_PRE_FAST             (6.2f)
#define STBC03J_CHG_OVER_CHRG            (8.2f)
#define STBC03J_CHG_TIMEOUT              (10.2f)
#define STBC03J_CHG_BEL_VPPRE            (12.8f)
#define STBC03J_CHG_THERMAL_WARNING      (14.2f)
#define STBC03J_CHG_NTC_WARNING          (16.2f)
#define CHECK_BOUNDS(a,b,th) ((a<(b+th))&&(a>(b-th)))


/** @defgroup BlueCoin_COMMON_LED BlueCoin_COMMON_LED
  * @{
  */

#define LEDn                             8

#define LED1_PIN                         GPIO_PIN_7
#define LED1_GPIO_PORT                   GPIOD
#define LED1_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LED2_PIN                         GPIO_PIN_4
#define LED2_GPIO_PORT                   GPIOD
#define LED2_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LED3_PIN                         GPIO_PIN_0
#define LED3_GPIO_PORT                   GPIOD
#define LED3_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LED4_PIN                         GPIO_PIN_13
#define LED4_GPIO_PORT                   GPIOD
#define LED4_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LED5_PIN                         GPIO_PIN_11
#define LED5_GPIO_PORT                   GPIOD
#define LED5_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED5_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LED6_PIN                         GPIO_PIN_4
#define LED6_GPIO_PORT                   GPIOA
#define LED6_GPIO_CLK_ENABLE()           __GPIOA_CLK_ENABLE()
#define LED6_GPIO_CLK_DISABLE()          __GPIOA_CLK_DISABLE()

#define LED7_PIN                         GPIO_PIN_15
#define LED7_GPIO_PORT                   GPIOC
#define LED7_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED7_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define LED8_PIN                         GPIO_PIN_6
#define LED8_GPIO_PORT                   GPIOD
#define LED8_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED8_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) LED1_GPIO_CLK_ENABLE(); else \
        if((__INDEX__) == 1) LED2_GPIO_CLK_ENABLE(); else \
            if((__INDEX__) == 2) LED3_GPIO_CLK_ENABLE(); else \
                if((__INDEX__) == 3) LED4_GPIO_CLK_ENABLE(); else \
                    if((__INDEX__) == 4) LED5_GPIO_CLK_ENABLE(); else \
                        if((__INDEX__) == 5) LED6_GPIO_CLK_ENABLE(); else \
                            if((__INDEX__) == 6) LED7_GPIO_CLK_ENABLE(); else \
                  if((__INDEX__) == 7) LED8_GPIO_CLK_ENABLE(); \
  }while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED1_GPIO_CLK_DISABLE(); else \
        if((__INDEX__) == 1) LED2_GPIO_CLK_DISABLE(); else \
            if((__INDEX__) == 2) LED3_GPIO_CLK_DISABLE(); else \
                if((__INDEX__) == 3) LED4_GPIO_CLK_DISABLE(); else \
                    if((__INDEX__) == 4) LED5_GPIO_CLK_DISABLE(); else \
                        if((__INDEX__) == 5) LED6_GPIO_CLK_DISABLE(); else \
                            if((__INDEX__) == 6) LED7_GPIO_CLK_DISABLE(); else \
                  if((__INDEX__) == 7) LED8_GPIO_CLK_DISABLE(); \
  }while(0)


/**
  * @}
  */

/** @defgroup BlueCoin_COMMON_BUTTON BlueCoin_COMMON_BUTTON
  * @{
  */
/* Joystick pins are connected to IO Expander (accessible through FMPI2C interface) */
#define BUTTONn                             ((uint8_t)2)

/**
  * @brief Wakeup push-button
  */
#define BUTTON_1_PIN                   GPIO_PIN_8
#define BUTTON_1_GPIO_PORT             GPIOE
#define BUTTON_1_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()
#define BUTTON_1_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOE_CLK_DISABLE()
#define BUTTON_1_EXTI_IRQn             EXTI9_5_IRQn

/**
  * @brief BUTTON2 push-button
  */
#define BUTTON_2_PIN                    GPIO_PIN_9
#define BUTTON_2_GPIO_PORT              GPIOE
#define BUTTON_2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()
#define BUTTON_2_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOE_CLK_DISABLE()
#define BUTTON_2_EXTI_IRQn              EXTI9_5_IRQn


    
#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == 0) {BUTTON_1_GPIO_CLK_ENABLE();} else\
    {BUTTON_2_GPIO_CLK_ENABLE();   }} while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    ((__INDEX__) == 0) ? BUTTON_1_GPIO_CLK_DISABLE() : BUTTON_2_GPIO_CLK_DISABLE())
/**
  * @}
  */


/** @defgroup BlueCoin_COMMON_CHARGER
  * @{
  */
  
#define CHRG_PIN                    GPIO_PIN_12
#define CHRG_GPIO_PORT              GPIOD
#define CHRG_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
#define CHRG_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOD_CLK_DISABLE()
#define CHRG_EXTI_IRQn              EXTI15_10_IRQn
/**
  * @}
  */


/** @defgroup BlueCoin_COMMON_SD BlueCoin_COMMON_SD
  * @{
  */
  
#define SD_CMD_GPIO_PORT           GPIOD
#define SD_CMD_GPIO_PIN            GPIO_PIN_2
#define SD_CMD_GPIO_CLK_ENABLE()   __GPIOD_CLK_ENABLE()
#define SD_CMD_GPIO_CLK_DISABLE()  __GPIOD_CLK_DISABLE()

#define SD_CLK_GPIO_PORT           GPIOB
#define SD_CLK_GPIO_PIN            GPIO_PIN_2
#define SD_CLK_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define SD_CLK_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()

#define SD_D0_GPIO_PORT            GPIOC
#define SD_D0_GPIO_PIN             GPIO_PIN_8
#define SD_D0_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()
#define SD_D0_GPIO_CLK_DISABLE()   __GPIOC_CLK_DISABLE()

#define SD_DETECT_GPIO_PORT           GPIOB
#define SD_DETECT_GPIO_PIN            GPIO_PIN_7
#define SD_DETECT_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define SD_DETECT_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define SD_DETECT_EXTI_IRQn           EXTI9_5_IRQn

#define SD_DUMMY_BYTE            0xFF
#define SD_NO_RESPONSE_EXPECTED  0x80

/**
  * @}
  */

/* Definition for ADC clock resources */
#define BATTERY_MONITOR_ADC                  ADC1
#define BATTERY_MONITOR_ADC_CLK_ENABLE()     __HAL_RCC_ADC1_CLK_ENABLE()

#define BATTERY_MONITOR_ADC_FORCE_RESET()    __HAL_RCC_ADC_FORCE_RESET()
#define BATTERY_MONITOR_ADC_RELEASE_RESET()  __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for Battery monitor Pin */
#define BATTERY_MONITOR_GPIO_PIN             GPIO_PIN_0
#define BATTERY_MONITOR_GPIO_PORT            GPIOC
#define BATTERY_MONITOR_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()

#define BATMS_EN_GPIO_PIN                    GPIO_PIN_14
#define BATMS_EN_GPIO_PORT                   GPIOB
#define BATMS_EN_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()

#define SHUTDOWN_GPIO_PIN                    GPIO_PIN_9
#define SHUTDOWN_GPIO_PORT                   GPIOC
#define SHUTDOWN_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
      
/* Definition for ADC's Channel */
#define BATTERY_MONITOR_ADC_CHANNEL          ADC_CHANNEL_10
#define SAMPLINGTIME                         ADC_SAMPLETIME_3CYCLES

/** @defgroup BlueCoin_COMMON_Private_Functions BlueCoin_COMMON_Public_Functions
* @{
*/
uint32_t         BSP_GetVersion(void);
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
void             BSP_Mic234_Clock_Selector_Init(void);
void             BSP_Mic234_Clock_Selector_Set(Mic234Clk_Status_TypeDef Mic234Clk_Status);
void             BSP_ShutDown_Init(void);
void             BSP_ShutDown(void);
void             BSP_Watchdog_Refresh(void);
void             BSP_ChrgPin_Init(void);
void             BSP_SetLastChrgTick(uint32_t msTick);
void             BSP_BatMS_Enable(void);
void             BSP_BatMS_Disable(void);
void             BSP_PROX_Pin_init(void);
void             BSP_PROX_Pin_On(void);
void             BSP_PROX_Pin_Off(void);
void             I2C_ErrorCallback_PROX(I2C_HandleTypeDef *hi2c);
void             Sensor_IO_Error( void );
uint32_t         BSP_PB_GetState(Button_TypeDef Button);
ChrgStatus_t     BSP_GetChrgStatus(void);
DrvStatusTypeDef BSP_Watchdog_Init(uint16_t ReloadN);
DrvStatusTypeDef BSP_BatMS_Init(void);
DrvStatusTypeDef BSP_BatMS_DeInit(void);
DrvStatusTypeDef BSP_GetVoltage(uint16_t *volt);
uint8_t BSP_isBattPlugged(void);
uint8_t BSP_BatMS_isEnable(void);

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

#endif /* __BLUECOIN_H */
