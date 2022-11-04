/**
  ******************************************************************************
  * @file    stm32f4xx_nucleo_USART.c
  * @author  Central Labs
  * @version V1.0.0
  * @date    26-May-2015
  * @brief   This file provides set of firmware functions to manage:
  *          - USART
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
#ifndef __STM32F4XX_BSP_USART_H
#define __STM32F4XX_BSP_USART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F4XX_NUCLEO
  * @{
  */

/** @addtogroup STM32F4XX_NUCLEO_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Exported_Types
  * @{
  */


/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** 
  * @brief Define for STM32F4XX_NUCLEO board  
  */ 
#if !defined (USE_STM32F4XX_NUCLEO)
 #define USE_STM32F4XX_NUCLEO
#endif

/** @addtogroup STM32F4XX_NUCLEO_LOW_LEVEL_LED
  * @{
  */

/**
  * @}
  */ 
  
/** @addtogroup STM32F4XX_NUCLEO_LOW_LEVEL_BUTTON
  * @{
  */  
                         

/**
  * @}
  */ 

/** @addtogroup STM32F4XX_NUCLEO_LOW_LEVEL_BUS
  * @{
  */
   
   #define USARTn                                      1

/* UART Baud Rate configuration */
#ifndef BSP_USART_BAUD_RATE
 #define BSP_USART_BAUD_RATE               921600
#endif /* BSP_USART_BAUD_RATE */

#define BSP_USART_RX_BUFFER_SIZE           (1024)
#define BSP_USART_TX_BUFFER_SIZE           (1024)


/**
 * @brief Definition for USART port, connected to USART3
 */ 
#define NUCLEO_USART                              USART2
#define BSP_USART_CLK_ENABLE()                 __USART2_CLK_ENABLE()
#define BSP_USART_CLK_DISABLE()                __USART2_CLK_DISABLE()
   
#define BSP_USART_FORCE_RESET()                __USART2_FORCE_RESET()
#define BSP_USART_RELEASE_RESET()              __USART2_RELEASE_RESET()  

#define BSP_USART_TX_PIN                       GPIO_PIN_2
#define BSP_USART_TX_GPIO_PORT                 GPIOA
#define BSP_USART_TX_GPIO_CLK_ENABLE()         __GPIOA_CLK_ENABLE()
#define BSP_USART_TX_GPIO_CLK_DISABLE()        __GPIOA_CLK_DISABLE()
#define BSP_USART_TX_AF                        GPIO_AF7_USART2		

#define BSP_USART_RX_PIN                       GPIO_PIN_3
#define BSP_USART_RX_GPIO_PORT                 GPIOA
#define BSP_USART_RX_GPIO_CLK_ENABLE()         __GPIOA_CLK_ENABLE()
#define BSP_USART_RX_GPIO_CLK_DISABLE()        __GPIOA_CLK_DISABLE()
#define BSP_USART_RX_AF                        GPIO_AF7_USART2			

#define BSP_USART_IRQn                         USART2_IRQn
#define USARTx_IRQHandler                      USART2_IRQHandler

#define BSP_USART_DMA                          DMA1
#define BSP_USART_DMA_CLK_ENABLE()             __DMA1_CLK_ENABLE();
#define BSP_USART_DMA_CLK_DISABLE()            __DMA1_CLK_DISABLE();
#define BSP_USART_DMA_STREAM_RX	              DMA1_Stream5			
#define BSP_USART_DMA_STREAM_TX                DMA1_Stream6	
#define BSP_USART_DMA_STREAM_TX_IRQn           DMA1_Stream6_IRQn
#define BSP_USART_DMA_STREAM_TX_IRQHandler     DMA1_Stream6_IRQHandler
#define BSP_USART_DMA_CHANNEL_RX 	          DMA_CHANNEL_4 			
#define BSP_USART_DMA_CHANNEL_TX               DMA_CHANNEL_4 			
			

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Exported_Macros
  * @{
  */  
/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Exported_Functions
  * @{
  */
void            BSP_USART_Init(void);
uint16_t        BSP_USART_CheckForNewData(void);
void            BSP_USART_SendData(uint16_t msg_size);
uint8_t*        BSP_USART_GetRxBuffer(void);
uint8_t*        BSP_USART_GetTxBuffer(void);
  
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

#endif /* __STM32F4XX_BSP_USART_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
