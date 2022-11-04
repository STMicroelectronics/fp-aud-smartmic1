/**
  ******************************************************************************
  * @file    usbd_audio_core.h
  * @author  SRA
  * @brief   header file for the usbd_audio_core.c file.
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

#ifndef __USBD_AUDIO_CDC_H_
#define __USBD_AUDIO_CDC_H_

#include "usbd_ioreq.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
* @{
*/

/** @defgroup USBD_AUDIO_CDC
* @brief This file is the Header file for USBD_msc.c
* @{
*/

/** @defgroup USBD_AUDIO_CDC_Exported_Defines
* @{
*/
#define TIMEOUT_VALUE                                 200
#define AUDIO_OUT_EP                                  0x01
#define USB_AUDIO_CONFIG_DESC_SIZ                     109
#define AUDIO_INTERFACE_DESC_SIZE                     9
#define USB_AUDIO_DESC_SIZ                            0x09
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07

#define AUDIO_DESCRIPTOR_TYPE                         0x21
#define USB_DEVICE_CLASS_AUDIO                        0x01
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02
#define AUDIO_PROTOCOL_UNDEFINED                      0x00
#define AUDIO_STREAMING_GENERAL                       0x01
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                          0x01
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06

#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0C
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07

#define AUDIO_CONTROL_MUTE                            0x0002

#define AUDIO_FORMAT_TYPE_I                           0x01
#define AUDIO_FORMAT_TYPE_III                         0x03

#define AUDIO_ENDPOINT_GENERAL                        0x01

#define AUDIO_REQ_GET_CUR                             0x81
//added
#define AUDIO_REQ_GET_MIN                             0x82
#define AUDIO_REQ_GET_MAX                             0x83
#define AUDIO_REQ_GET_RES                             0x84

#define AUDIO_REQ_SET_CUR                             0x01
#define AUDIO_OUT_STREAMING_CTRL                      0x02

#define VOL_MIN  0xDBE0
#define VOL_RES  0x0023
#define VOL_MAX  0x0000

#define AUDIO_IN_PACKET                  (uint32_t)((((48000/1000)+2)*8)*2)

#define MIC_IN_TERMINAL_ID                            1
#define MIC_FU_ID                                     2
#define MIC_OUT_TERMINAL_ID                           3
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04


#define AUDIO_IN_EP                     0x83 /* Audio Data in endpoint */

/* Number of sub-packets in the audio transfer buffer. You can modify this value but always make sure
that it is an even number and higher than 3 */
#define AUDIO_IN_PACKET_NUM                            15
/* Total size of the audio transfer buffer */

/* Buffering state definitions */
typedef enum
{
  STATE_USB_WAITING_FOR_INIT = 0,
  STATE_USB_IDLE = 1,
  STATE_USB_REQUESTS_STARTED = 2,  
  STATE_USB_BUFFER_WRITE_STARTED = 3,    
}AUDIO_StatesTypeDef;
/* Audio Commands enmueration */
typedef enum
{
  AUDIO_CMD_START = 1,
  AUDIO_CMD_PLAY,
  AUDIO_CMD_STOP,
} 
AUDIO_CMD_TypeDef;


typedef enum
{
  AUDIO_OFFSET_NONE = 0,
  AUDIO_OFFSET_HALF,
  AUDIO_OFFSET_FULL,
  AUDIO_OFFSET_UNKNOWN,
}
AUDIO_OffsetTypeDef;

/**
* @}
*/


/** @defgroup USBD_CORE_Exported_TypesDefinitions
* @{
*/
typedef struct
{
  uint8_t cmd;
  uint8_t data[USB_MAX_EP0_SIZE];
  uint8_t len;
  uint8_t unit;
}
USBD_AUDIO_ControlTypeDef;

typedef struct
{
  __IO uint32_t              alt_setting;  
  uint8_t                    channels;
  uint16_t                   frequency;
  __IO int16_t                   timeout;
  uint16_t                   buffer_length;    
  uint16_t                   dataAmount;
  uint16_t                   paketDimension;   
  uint8_t                    state;  
  uint16_t                   rd_ptr;  
  uint16_t                   wr_ptr;  
  uint8_t                    upper_treshold;
  uint8_t                    lower_treshold;
  USBD_AUDIO_ControlTypeDef control;   
  uint8_t  *                 buffer;
}
USBD_AUDIO_HandleTypeDef; 


/**
* @}
*/



/** @defgroup USBD_CORE_Exported_Macros
* @{
*/

/**
* @}
*/

/** @defgroup USBD_CORE_Exported_Variables
* @{
*/

extern USBD_ClassTypeDef  USBD_AUDIO_CDC;
/**
* @}
*/

/** @defgroup USB_CORE_Exported_Functions
* @{
*/

/**
* @}
*/
/** @defgroup usbd_cdc_Exported_Defines
  * @{
  */
#define CDC_IN_EP                       0x81  /* EP1 for data IN */
#define CDC_OUT_EP                      0x01  /* EP1 for data OUT */
#define CDC_CMD_EP                      0x82  /* EP2 for CDC commands */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define CDC_DATA_HS_MAX_PACKET_SIZE        512  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE         64  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                  8  /* Control Endpoint Packet size */

#define USB_CDC_CONFIG_DESC_SIZ                67
#define CDC_DATA_HS_IN_PACKET_SIZE                CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE               CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE               CDC_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/

#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23

#define USB_HID_CDC_CONFIG_DESC_SIZ (182)
#define CDC_COM_INTERFACE 0x2
#define AUDIO_INTERFACE  0x0

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} USBD_CDC_LineCodingTypeDef;

typedef struct
{
  int8_t  (*Init_Audio)         (uint32_t  AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
  int8_t  (*Init_CDC)           (void);
  int8_t  (*DeInit)       	(uint32_t options);
  int8_t  (*Record)     	(void);
  int8_t  (*AudioCtl)    	(uint8_t, uint8_t * , uint16_t);
  int8_t  (*Stop)   		(void);
  int8_t  (*Pause)   		(void);
  int8_t  (*Resume)   		(void);
  int8_t  (*CommandMgr)         (uint8_t cmd);
  int8_t (* CDCCtl)             (uint8_t, uint8_t * , uint16_t);
  int8_t (* Receive)            (uint8_t *, uint32_t *);
}USBD_AUDIO_CDC_ItfTypeDef;


typedef struct
{
  uint32_t data[CDC_DATA_HS_MAX_PACKET_SIZE / 4];    /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;
  
  __IO uint32_t TxState;
  __IO uint32_t RxState;
}
USBD_CDC_HandleTypeDef;



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_CDC;
#define USBD_CDC_CLASS    &USBD_CDC
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_AUDIO_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev,
    USBD_AUDIO_CDC_ItfTypeDef *fops);
uint8_t  USBD_AUDIO_Data_Transfer (USBD_HandleTypeDef *pdev, int16_t * audioData, uint16_t dataAmount);


uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length);

uint8_t  USBD_CDC_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);

uint8_t  USBD_CDC_ReceivePacket  (USBD_HandleTypeDef *pdev);

uint8_t  USBD_CDC_TransmitPacket  (USBD_HandleTypeDef *pdev);

uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                        USBD_AUDIO_CDC_ItfTypeDef *fops);
void USBD_AUDIO_Sync (USBD_HandleTypeDef *pdev, int16_t * audioData);
void USBD_AUDIO_CDC_Init_Microphone_Descriptor(uint32_t samplingFrequency, uint8_t Channels);

#endif  // __USB_AUDIO_IN_H_
/**
* @}
*/

/**
* @}
*/

