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
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_nucleo_USART.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32F4XX_NUCLEO
  * @{
  */   
    
/** @addtogroup STM32F4XX_NUCLEO_LOW_LEVEL 
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32F4xx-Nucleo Kit from STMicroelectronics.
  * @{
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_Defines
  * @{
  */ 

/**
  * @brief STM32F4xx NUCLEO BSP Driver version number V1.2.1
  */



/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_Variables
  * @{
  */ 

volatile uint8_t BSP_USART_RxBuffer[BSP_USART_RX_BUFFER_SIZE];

volatile uint8_t BSP_USART_TxBuffer[BSP_USART_TX_BUFFER_SIZE];

UART_HandleTypeDef   BSP_USART_Handle;
/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_Functions
  * @{
  */ 



/******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
/**
  * @brief  Configures the current time and date.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void)
{
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  BSP_USART_Handle.Instance        = NUCLEO_USART;

  BSP_USART_Handle.Init.BaudRate   = BSP_USART_BAUD_RATE;
  BSP_USART_Handle.Init.WordLength = UART_WORDLENGTH_8B;
  BSP_USART_Handle.Init.StopBits   = UART_STOPBITS_1;
  BSP_USART_Handle.Init.Parity     = UART_PARITY_NONE;
  BSP_USART_Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  BSP_USART_Handle.Init.Mode         = UART_MODE_TX_RX;
  BSP_USART_Handle.Init.OverSampling = UART_OVERSAMPLING_16;

  BSP_USART_Handle.pRxBuffPtr      = (uint8_t *)&BSP_USART_RxBuffer[0];
  BSP_USART_Handle.RxXferSize      = BSP_USART_RX_BUFFER_SIZE;
  BSP_USART_Handle.pTxBuffPtr      = (uint8_t *)&BSP_USART_TxBuffer[0];
  BSP_USART_Handle.TxXferSize      = BSP_USART_TX_BUFFER_SIZE;

  HAL_UART_Init(&BSP_USART_Handle);

  HAL_UART_Receive_DMA(&BSP_USART_Handle, BSP_USART_Handle.pRxBuffPtr, BSP_USART_RX_BUFFER_SIZE);
}


/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  static DMA_HandleTypeDef    BSP_USART_hdma_tx;
  static DMA_HandleTypeDef    BSP_USART_hdma_rx;
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock */
  BSP_USART_TX_GPIO_CLK_ENABLE();
  BSP_USART_RX_GPIO_CLK_ENABLE();
  /* Enable USART3 clock */
  BSP_USART_CLK_ENABLE(); 
  /* Enable DMA1 clock */
  BSP_USART_DMA_CLK_ENABLE();   
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = BSP_USART_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = BSP_USART_TX_AF;
  
  HAL_GPIO_Init(BSP_USART_TX_GPIO_PORT, &GPIO_InitStruct);
    
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = BSP_USART_RX_PIN;
  GPIO_InitStruct.Alternate = BSP_USART_RX_AF;
    
  HAL_GPIO_Init(BSP_USART_RX_GPIO_PORT, &GPIO_InitStruct);
    
  /*##-3- Configure the DMA streams ##########################################*/
  /* Configure the DMA handler for Transmission process */
  BSP_USART_hdma_tx.Instance                 = BSP_USART_DMA_STREAM_TX;
  
  BSP_USART_hdma_tx.Init.Channel             = BSP_USART_DMA_CHANNEL_TX;
  BSP_USART_hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  BSP_USART_hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  BSP_USART_hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  BSP_USART_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  BSP_USART_hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  BSP_USART_hdma_tx.Init.Mode                = DMA_NORMAL;
  BSP_USART_hdma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
  BSP_USART_hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  BSP_USART_hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  BSP_USART_hdma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
  BSP_USART_hdma_tx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
  
  HAL_DMA_Init(&BSP_USART_hdma_tx);   
  
  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA(huart, hdmatx, BSP_USART_hdma_tx);
    
  /* Configure the DMA handler for reception process */
  BSP_USART_hdma_rx.Instance                 = BSP_USART_DMA_STREAM_RX;
  
  BSP_USART_hdma_rx.Init.Channel             = BSP_USART_DMA_CHANNEL_RX;
  BSP_USART_hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  BSP_USART_hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  BSP_USART_hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  BSP_USART_hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  BSP_USART_hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  BSP_USART_hdma_rx.Init.Mode                = DMA_CIRCULAR;
  BSP_USART_hdma_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
  BSP_USART_hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  BSP_USART_hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  BSP_USART_hdma_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
  BSP_USART_hdma_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE; 

  HAL_DMA_Init(&BSP_USART_hdma_rx);
    
  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(huart, hdmarx, BSP_USART_hdma_rx);
      
  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (USART3_TX) */
  HAL_NVIC_SetPriority(BSP_USART_DMA_STREAM_TX_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(BSP_USART_DMA_STREAM_TX_IRQn);
  
    HAL_NVIC_SetPriority(BSP_USART_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(BSP_USART_IRQn);
}


/**
 * @brief  Check if new dara are available
 * @param  None
 * @retval None
 */
uint16_t BSP_USART_CheckForNewData(void)
{
 return (uint16_t)( BSP_USART_Handle.RxXferSize - ((uint16_t)__HAL_DMA_GET_COUNTER(BSP_USART_Handle.hdmarx)));
}

/**
 * @brief  Send data via UART
 * @param  msg_size: size of the msg to be sent
 * @retval None
 */
void BSP_USART_SendData(uint16_t msg_size)
{
  BSP_USART_Handle.TxXferSize = msg_size;
  BSP_USART_Handle.TxXferCount = msg_size;
  HAL_UART_Transmit_DMA(&BSP_USART_Handle, BSP_USART_Handle.pTxBuffPtr, msg_size);
}
	
/**
 * @brief  Return the UART Rx buffer
 * @param  None
 * @retval UART Rx buffer
 */
uint8_t* BSP_USART_GetRxBuffer(void)
{
  return (uint8_t *)BSP_USART_RxBuffer;
}

/**
 * @brief  Return the UART Tx buffer
 * @param  None
 * @retval UART Tx buffer
 */
uint8_t* BSP_USART_GetTxBuffer(void)
{
  return (uint8_t *)BSP_USART_TxBuffer;
}

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//  /* Transfer in transmission process is correct */
//  huart->State = HAL_UART_STATE_READY;
//  
//}


/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/



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
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
