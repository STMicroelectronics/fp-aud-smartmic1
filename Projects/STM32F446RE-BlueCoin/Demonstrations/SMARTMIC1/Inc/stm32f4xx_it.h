/**
******************************************************************************
* @file    stm32f4xx_it.h
* @author  SRA
* 
* 
* @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif 
  
  /* Includes ------------------------------------------------------------------*/
#include "main.h"
  
  /* Exported types ------------------------------------------------------------*/
  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */
  extern I2S_HandleTypeDef                hAudioInI2s;
  extern SAI_HandleTypeDef hAudioOutSai;
  extern UART_HandleTypeDef   BSP_USART_Handle;
  

  void NMI_Handler(void);
  void HardFault_Handler(void);
  void MemManage_Handler(void);
  void BusFault_Handler(void);
  void UsageFault_Handler(void);
  void SVC_Handler(void);
  void DebugMon_Handler(void);
  void PendSV_Handler(void);
  void SysTick_Handler(void);
  
  void AUDIO_IN_I2S_IRQHandler(void);
  void OTG_FS_IRQHandler(void);
  void EXTI9_5_IRQHandler(void);
  void TIM_USB_IRQHandler(void);
  void EXTI1_IRQHandler(void);
  void EXTI2_IRQHandler(void);
  void EXTI3_IRQHandler(void);
  void EXTI4_IRQHandler(void);
  
  void BLUECOINPLUS_I2C_EV_IRQHandler(void);
  void BLUECOINPLUS_I2C_ER_IRQHandler(void);
  
  void DMA2_Stream1_IRQHandler(void);
  void DMA1_Stream4_IRQHandler(void);
  void DMA1_Stream3_IRQHandler(void);
  void DMA1_Stream7_IRQHandler(void);
  
  
  void BSP_USART_DMA_STREAM_TX_IRQHandler(void);
  void USARTx_IRQHandler(void);
  
  
  
  
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */


