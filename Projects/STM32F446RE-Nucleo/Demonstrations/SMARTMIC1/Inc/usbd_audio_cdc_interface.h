/**
******************************************************************************
* @file    USB_Device/CDC_Standalone/Inc/usbd_cdc_interface.h
* @author  SRA
* 
* 
* @brief   Header for usbd_cdc_interface.c file.
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
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_cdc.h"
#ifdef USE_STM32F4XX_NUCLEO
 #include "cca02m2_audio.h"
#else
 #include "BlueCoin_audio_in.h"
 #include "BlueCoin_audio_out.h"
#endif  
  
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */

#define USB_RxBufferDim                         512

/* Definition for TIMx clock resources */
#define TIMx                             TIM4
#define TIMx_CLK_ENABLE                  __TIM4_CLK_ENABLE
#define TIMx_FORCE_RESET()               __USART3_FORCE_RESET()
#define TIMx_RELEASE_RESET()             __USART3_RELEASE_RESET()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                        TIM4_IRQn
#define TIM_USB_IRQHandler               TIM4_IRQHandler

/* Periodically, the state of the buffer "UserTxBuffer" is checked.
   The period depends on CDC_POLLING_INTERVAL */
#define CDC_POLLING_INTERVAL             15 /* in ms. The max is 65ms and the min is 1ms */

extern USBD_AUDIO_CDC_ItfTypeDef  USBD_AUDIO_CDC_fops;
extern TIM_HandleTypeDef  TimHandle;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t CDC_Fill_Buffer(uint8_t* Buf, uint32_t TotalLen);
uint16_t USB_CheckForNewData(void);
uint8_t* USB_GetRxBuffer(void);
uint8_t* USB_GetTxBuffer(void);

#define AUDIO_REQ_GET_CUR                           0x81
#define AUDIO_REQ_GET_MIN                           0x82
#define AUDIO_REQ_GET_MAX                           0x83
#define AUDIO_REQ_GET_RES                           0x84

int8_t Audio_VolumeCtl(int16_t Volume);
void Send_Audio_to_USB(int16_t * audioData, uint16_t PCMSamples);


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H */


