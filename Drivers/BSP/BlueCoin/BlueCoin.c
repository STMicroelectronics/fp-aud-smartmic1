/**
******************************************************************************
* @file    BlueCoin.c
* @author  SRA - Central Labs
* @version v2.2.3
* @date    10-Feb-2022
* @brief   This file provides low level functionalities for BlueCoin board.
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


/* File Info: ------------------------------------------------------------------


------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "BlueCoin.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_exti.h"

/** @addtogroup BSP
* @{
*/

/** @defgroup BlueCoin
* @{
*/

/** @defgroup BlueCoin_COMMON BlueCoin_COMMON
* @{
*/


/** @defgroup BlueCoin_COMMON_Private_Defines BlueCoin_COMMON_Private_Defines
* @{
*/


/**
* @brief BlueCoin BSP Driver version number v2.2.3
*/
#define __BlueCoin_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __BlueCoin_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __BlueCoin_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __BlueCoin_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __BlueCoin_BSP_VERSION         ((__BlueCoin_BSP_VERSION_MAIN << 24)\
                                        |(__BlueCoin_BSP_VERSION_SUB1 << 16)\
                                        |(__BlueCoin_BSP_VERSION_SUB2 << 8 )\
                                        |(__BlueCoin_BSP_VERSION_RC))



/**
* @}
*/


I2C_HandleTypeDef    I2C_ONBOARD_SENSORS_Handle;
ADC_HandleTypeDef    AdcHandle;
IWDG_HandleTypeDef   IwdgHandle;


/** @defgroup BlueCoin_COMMON_Private_Variables BlueCoin_COMMON_Private_Variables
* @{
*/
static uint32_t msLastTicks[2] = {1,1};
volatile static ChrgStatus_t ChrgStatus;
volatile uint8_t IT_I2C_error_af=0;					
__IO uint16_t uhADCxConvertedValue = 0;
volatile static uint8_t BatMSEnable=0;

GPIO_TypeDef* GPIO_PORT[LEDn] =
{
  LED1_GPIO_PORT,
  LED2_GPIO_PORT,
  LED3_GPIO_PORT,
  LED4_GPIO_PORT,
  LED5_GPIO_PORT,
  LED6_GPIO_PORT,
  LED7_GPIO_PORT,
  LED8_GPIO_PORT
};

const uint32_t GPIO_PIN[LEDn] =
{
  LED1_PIN,
  LED2_PIN,
  LED3_PIN,
  LED4_PIN,
  LED5_PIN,
  LED6_PIN,
  LED7_PIN,
  LED8_PIN
};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] =
{
  BUTTON_1_GPIO_PORT,
  BUTTON_2_GPIO_PORT
};

const uint16_t BUTTON_PIN[BUTTONn] =
{
  BUTTON_1_PIN,
  BUTTON_2_PIN
};

const uint16_t BUTTON_IRQn[BUTTONn] =
{
  BUTTON_1_EXTI_IRQn,
  BUTTON_2_EXTI_IRQn
};

/**
* @}
*/

static void                     ADC_BLUECOIN_MspInit(void);
static void                     ADC_BLUECOIN_MspDeInit(void);
void                            LSM6DSM_Sensor_IO_ITConfig( void );


/** @defgroup BlueCoin_COMMON_Private_Functions BlueCoin_COMMON_Public_Functions
* @{
*/

/**
* @brief  This method returns the BlueCoin EVAL BSP Driver revision
* @param  None
* @retval version: 0xXYZR (8bits for each decimal, R for RC)
*/
uint32_t BSP_GetVersion(void)
{
  return __BlueCoin_BSP_VERSION;
}

/**
* @brief  This function confgures the GPIO which manages the shutdown pin
* @param  None
* @retval version:
*/
void BSP_ShutDown_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED clock */
  SHUTDOWN_GPIO_CLK_ENABLE();
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = SHUTDOWN_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  /* Reset the pin before init to prevent glitches */
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_Init(SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}

/**
* @brief  This function manages the shutdown pin of STBC03J turning off the board
* @param  None
* @retval version
*/
void BSP_ShutDown(void)
{
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_GPIO_PIN, GPIO_PIN_SET);
}


/**
* @brief Confgure the STM32 internal watchdog. IWDG counter clock Period
* is set to 4ms, watchdog timeout is 4ms*(ReloadN+1).
* @param  ReloadN: IWDG down-counter reload value [0-4095]
* @retval version:
*/
DrvStatusTypeDef BSP_Watchdog_Init(uint16_t ReloadN)
{
  /* Set counter reload value to obtain 4ms*(ReloadN+1) IWDG TimeOut.
     IWDG counter clock Period = 1 / (LsiFreq / Prescaler) = 1/(32KHz/128) = 4ms */
  IwdgHandle.Instance = IWDG;

  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_128;
  IwdgHandle.Init.Reload    = (ReloadN&0xFFF);

  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    /* Initialization Error */
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
* @brief  Refresh the STM32 internal watchdog
* @param  none
* @retval none
*/
void BSP_Watchdog_Refresh()
{
  HAL_IWDG_Refresh(&IwdgHandle);
}


/**
* @brief  This method init the GPIO for charger status routine
* @param  None
* @retval none
*/
void BSP_ChrgPin_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED clock */
  CHRG_GPIO_CLK_ENABLE();
 
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = CHRG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(CHRG_GPIO_PORT, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority(CHRG_EXTI_IRQn, 3, 0x00);
  HAL_NVIC_EnableIRQ(CHRG_EXTI_IRQn);
}

/**
* @brief  This method get the charger status of the board
* @param  None
* @retval Charger status
*/
ChrgStatus_t BSP_GetChrgStatus(void)
{  
  uint32_t timeElapsed = HAL_GetTick()-msLastTicks[0];
 
  if(timeElapsed > 1000 && HAL_GPIO_ReadPin(CHRG_GPIO_PORT,CHRG_PIN) == GPIO_PIN_SET)
  {
    ChrgStatus = CHRG_STATUS_DISCHARGING;
  }
  return ChrgStatus;
}

/**
* @brief Update board charging status value
* @param  system tick
* @retval None
*/
void BSP_SetLastChrgTick(uint32_t msTick)
{
  float freqs[2];
  
  freqs[0] = 1000/(((float)msTick-(float)msLastTicks[0]));
  freqs[1] = 1000/((float)msLastTicks[0]-(float)msLastTicks[1]);
  
  msLastTicks[1]=msLastTicks[0];
  msLastTicks[0]=msTick;
    
  if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_EOC,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_EOC,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_EOC;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_PRE_FAST,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_PRE_FAST,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_CHARGING;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_OVER_CHRG,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_OVER_CHRG,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_OVER_CHRG;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_TIMEOUT,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_TIMEOUT,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_TIMEOUT;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_BEL_VPPRE,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_BEL_VPPRE,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_BEL_VPPRE;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_THERMAL_WARNING,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_THERMAL_WARNING,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_THERMAL_WARNING;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_NTC_WARNING,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_NTC_WARNING,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_NTC_WARNING;
  }
  else
  {
    ChrgStatus = CHRG_STATUS_DISCHARGING;
  }
  
}

/**
* @brief  This method reset the peripherals used to get the current voltage of battery
* @param  None
* @retval BSP_ERROR_NONE in case of success
* @retval BSP_ERROR_COMPONENT_FAILURE in case of failures
*/
DrvStatusTypeDef BSP_BatMS_DeInit(void)
{
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    return COMPONENT_ERROR;
  }
  ADC_BLUECOIN_MspDeInit();
  
  return COMPONENT_OK;
  
}

/**
* @brief  This method initializes the peripherals used to get the current voltage of battery
* @param  None
* @retval BSP_ERROR_NONE in case of success
* @retval BSP_ERROR_COMPONENT_FAILURE in case of failures
*/
DrvStatusTypeDef BSP_BatMS_Init(void)
{
  /* Variable used to get converted value */
  ADC_ChannelConfTypeDef sConfig;
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  BATMS_EN_GPIO_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = BATMS_EN_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_WritePin(BATMS_EN_GPIO_PORT, BATMS_EN_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_Init(BATMS_EN_GPIO_PORT, &GPIO_InitStruct); 
  
   /*##-1- Configure the ADC peripheral #######################################*/
  AdcHandle.Instance          = BATTERY_MONITOR_ADC;
  
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    return COMPONENT_ERROR;
  }

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;   /* Asynchronous clock mode, input ADC clock not divided */
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;            /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode          = DISABLE;                        /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.EOCSelection          = DISABLE;                        /* EOC flag picked-up to indicate conversion end */
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                        /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  AdcHandle.Init.NbrOfConversion       = 1;                              /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                        /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 0;                              /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;    /* Software start to trig the 1st conversion manually, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;  /* Parameter discarded because software trigger chosen */
  AdcHandle.Init.DMAContinuousRequests = DISABLE;                        /* DMA one-shot mode selected (not applied to this example) */
  
  
  ADC_BLUECOIN_MspInit();
  
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    return COMPONENT_ERROR;
  }
  
  /*##-2- Configure ADC regular channel ######################################*/
  sConfig.Channel      = BATTERY_MONITOR_ADC_CHANNEL;                /* Sampled channel number */
  sConfig.Rank         = 1;          /* Rank of sampled channel number BATTERY_MONITOR_ADC_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;    /* Sampling time (number of clock cycles unit) */
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
* @brief  This method enables the sensing of the current voltage of battery
* @param  None
* @retval None
*/
void BSP_BatMS_Enable(void)
{
  HAL_GPIO_WritePin(BATMS_EN_GPIO_PORT, BATMS_EN_GPIO_PIN, GPIO_PIN_SET);
  BatMSEnable=1;
}

/**
* @brief  This method disable the sensing of the current voltage of battery
* @param  None
* @retval None
*/
void BSP_BatMS_Disable(void)
{
  HAL_GPIO_WritePin(BATMS_EN_GPIO_PORT, BATMS_EN_GPIO_PIN, GPIO_PIN_RESET);
  BatMSEnable=0;
}

/**
* @brief  This method enables the sensing of the current voltage of battery
* @param  None
* @retval None
*/
uint8_t BSP_BatMS_isEnable(void)
{
  return BatMSEnable;
}

/**
* @brief  This method gets the current voltage of battery
* @param  volt pointer to destination variable
* @retval BSP_ERROR_NONE in case of success
* @retval BSP_ERROR_COMPONENT_FAILURE in case of failures
*/
DrvStatusTypeDef BSP_GetVoltage(uint16_t *volt)
{

  if(!BSP_BatMS_isEnable())
  {
    BSP_BatMS_Enable();
    HAL_Delay(1);
  }
  /*##-3- Start the conversion process #######################################*/
  if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    /* Start Conversation Error */
    return COMPONENT_ERROR;
  }
  
  /*##-4- Wait for the end of conversion #####################################*/
  /*  Before starting a new conversion, you need to check the current state of
  the peripheral; if it’s busy you need to wait for the end of current
  conversion before starting a new one.
  For simplicity reasons, this example is just waiting till the end of the
  conversion, but application may perform other tasks while conversion
  operation is ongoing. */
  if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK)
  {
    /* End Of Conversion flag not set on time */
    return COMPONENT_ERROR;
  }
  
  /* Check if the continuous conversion of regular channel is finished */
  if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
  {
    /*##-5- Get the converted value of regular channel  ########################*/
    uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
  }
  
  if(BSP_BatMS_isEnable())
  {
    BSP_BatMS_Disable();
  }
  
  
  uint32_t Voltage;
  Voltage = (3000 * (uint32_t)uhADCxConvertedValue) / (4095);  // [0-3V]
  Voltage = ((56+133)*Voltage)/133;   // [0-4.2V]
  
  *volt= Voltage;
  
  return COMPONENT_OK;
}

/**
  * @brief ADC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param None
  * @retval None
  */
void ADC_BLUECOIN_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* ADC Periph clock enable */
  BATTERY_MONITOR_ADC_CLK_ENABLE();
  /* Enable GPIO clock ****************************************/
  BATTERY_MONITOR_GPIO_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/
  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = BATTERY_MONITOR_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BATTERY_MONITOR_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief ADC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param None
  * @retval None
  */
void ADC_BLUECOIN_MspDeInit(void)
{

  /*##-1- Reset peripherals ##################################################*/
  BATTERY_MONITOR_ADC_FORCE_RESET();
  BATTERY_MONITOR_ADC_RELEASE_RESET();
  
  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC Channel GPIO pin */
  HAL_GPIO_DeInit(BATTERY_MONITOR_GPIO_PORT, BATTERY_MONITOR_GPIO_PIN);
}

/**
* @brief  Configures LEDs.
* @param  Led: LED to be configured.
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @retval None
*/
void BSP_LED_Init(Led_TypeDef Led)
{

  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}

/**
* @brief  DeInit LEDs.
* @param  Led: LED to be configured.
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @note Led DeInit does not disable the GPIO clock nor disable the Mfx
* @retval None
*/
void BSP_LED_DeInit(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
    LEDx_GPIO_CLK_DISABLE(Led);
}

/**
* @brief  Turns selected LED On.
* @param  Led: LED to be set on
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @retval None
*/
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
* @brief  Turns selected LED Off.
* @param  Led: LED to be set off
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @retval None
*/
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}


/**
* @brief  Toggles the selected LED.
* @param  Led: LED to be toggled
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @retval None
*/
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
* @brief  Configures button GPIO and EXTI Line.
* @param  Button: Button to be configured
*          This parameter can be one of the following values:
*            @arg  BUTTON_1: Push Button 1
*            @arg  BUTTON_2: Push Button 2
* @param  ButtonMode: Button mode
*          This parameter can be one of the following values:
*            @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
*            @arg  BUTTON_MODE_EXTI: Button will be connected to EXTI line
*                                    with interrupt generation capability
* @retval None
*/
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpio_init_structure;
  
  /* Enable the BUTTON clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
  
  if(ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    gpio_init_structure.Pin = BUTTON_PIN[Button];
    gpio_init_structure.Mode = GPIO_MODE_INPUT;
    gpio_init_structure.Pull = GPIO_PULLDOWN;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpio_init_structure);
  }
  
  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    gpio_init_structure.Pin = BUTTON_PIN[Button];
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    if(Button == BUTTON_1)
    {
      gpio_init_structure.Pull = GPIO_PULLDOWN;
      gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
    }
    else if (Button == BUTTON_2)
    {
      gpio_init_structure.Pull = GPIO_NOPULL;
      gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
    }
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpio_init_structure);
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
* @brief  Push Button DeInit.
* @param  Button: Button to be configured
*          This parameter can be one of the following values:
*            @arg  BUTTON_WAKEUP: Wakeup Push Button
*            @arg  BUTTON_TAMPER: Tamper Push Button
*            @arg  BUTTON_KEY: Key Push Button
* @note PB DeInit does not disable the GPIO clock
* @retval None
*/
void BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef gpio_init_structure;
  
  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);
}


/**
* @brief  Returns the selected button state.
* @param  Button: Button to be checked
*          This parameter can be one of the following values:
*            @arg  BUTTON_WAKEUP: Wakeup Push Button
*            @arg  BUTTON_TAMPER: Tamper Push Button
*            @arg  BUTTON_KEY: Key Push Button
* @retval The Button GPIO pin value
*/
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
* @brief  Configures GPIO Mics1234 Source Selector.
* @param  None
* @retval None
*/
void BSP_Mic234_Clock_Selector_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable the Mics123SourceSelector Clock */
  MIC234_CLK_EN_GPIO_CLK_ENABLE();
  
  /* Configure the Mics123SourceSelector pin */
  GPIO_InitStruct.Pin = MIC234_CLK_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(MIC234_CLK_EN_GPIO_PORT, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(MIC234_CLK_EN_GPIO_PORT, MIC234_CLK_EN_PIN, GPIO_PIN_SET);
}

/**
* @brief  Mics 1234 Source switch.
* @param  Mics1234Source: Specifies the Mics1234 Source to be configured.
*   This parameter can be one of following parameters:
*     @arg ONBOARD
*     @arg EXTERNAL
* @retval None
*/
void BSP_Mic234_Clock_Selector_Set(Mic234Clk_Status_TypeDef Mic234Clk_Status)
{
  if (Mic234Clk_Status == CLK_ENABLE)
  {
    HAL_GPIO_WritePin(MIC234_CLK_EN_GPIO_PORT, MIC234_CLK_EN_PIN, GPIO_PIN_SET);
  }
  if (Mic234Clk_Status == CLK_DISABLE)
  {
    HAL_GPIO_WritePin(MIC234_CLK_EN_GPIO_PORT, MIC234_CLK_EN_PIN, GPIO_PIN_RESET);
  }
}


uint8_t BSP_isBattPlugged()
{
  uint16_t voltage;
  static int i=0;
  static float delta=0.0f;
  static float delta2=0.0f;
  static float mean=0.0f;
  static float M2=0.0f;
  static uint8_t flag =0;
  
  if(i++ <= 100)
  {
    BSP_GetVoltage(&voltage);
    delta= (float)voltage - mean;
    mean= mean +delta/i;
    delta2= (float)voltage -mean;
    M2= M2+ delta*delta2;
  }
  else
  {
    flag= ((M2/(i-1)) < 10000.0f) ? 1 : 0;
    i=0;
    delta=0.0f;
    delta2=0.0f;
    mean=0.0f;
    M2=0.0f;
   
  }
  return flag;
 
}

/**
 * @brief  Configures shutdown pin to switch on the VL53L0x on CoinStation expansion
 * @retval None
 */
void BSP_PROX_Pin_init()
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  __GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
}

/**
 * @brief  Switch on the VL53L0x on CoinStation expansion
 * @retval None
 */
void BSP_PROX_Pin_On()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
}

/**
 * @brief  Switch off the VL53L0x on CoinStation expansion
 * @retval None
 */
void BSP_PROX_Pin_Off()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}



/**
  * @brief  I2C error callback fot interrupt mode comunication.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void I2C_ErrorCallback_PROX(I2C_HandleTypeDef *hi2c)
{
  uint32_t er=0x00;
  
  er= HAL_I2C_GetError(hi2c);
  
  if(er == HAL_I2C_ERROR_AF)
  {
    IT_I2C_error_af=1;
  }
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
