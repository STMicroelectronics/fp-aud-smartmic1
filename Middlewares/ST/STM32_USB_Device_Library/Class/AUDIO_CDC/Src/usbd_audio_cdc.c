/**
  ******************************************************************************
  * @file    usbd_audio_cdc.c
  * @author  SRA
  * @brief   This file provides the Audio Input core functions.
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
#include "usbd_audio_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
* @{
*/
/* This is the pointer used by the write process (from host to device) */
/* This dummy buffer with 0 values will be sent when there is no availble data */
static uint8_t IsocInBuffDummy[48 * 8 * 2]; 
static int16_t VOL_CUR;
static USBD_AUDIO_HandleTypeDef   haudioInstance;

/** @defgroup USBD_AUDIO_CDC
* @brief usbd core module
* @{
*/

/** @defgroup USBD_AUDIO_CDC_Private_TypesDefinitions
* @{
*/

/**
* @}
*/

/** @defgroup USBD_AUDIO_CDC_Private_Defines
* @{
*/

/**
* @}
*/

/** @defgroup USBD_AUDIO_CDC_Private_Macros
* @{
*/

/**
* @}
*/

/** @defgroup USBD_AUDIO_CDC_Private_FunctionPrototypes
* @{
*/

/*AUDIO CALLBACKS DECLARATIONS*/
static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMaximum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMinimum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetResolution(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/*CDC CALLBACKS DECLARATIONS*/
/** @defgroup USBD_CDC_Private_FunctionPrototypes
* @{
*/

static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);
uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor (uint16_t *length);

/*COMPOSITE CALLBACKS DECLARATIONS*/
static uint8_t  USBD_AUDIO_CDC_Init         (USBD_HandleTypeDef *pdev , uint8_t cfgidx);
static uint8_t  USBD_AUDIO_CDC_DeInit       (USBD_HandleTypeDef *pdev , uint8_t cfgidx);
static uint8_t  USBD_AUDIO_CDC_Setup        (USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req);
static uint8_t  USBD_AUDIO_CDC_EP0_RxReady  (USBD_HandleTypeDef *pdev );
static uint8_t  USBD_AUDIO_CDC_DataIn       (USBD_HandleTypeDef *pdev , uint8_t epnum);
static uint8_t  USBD_AUDIO_CDC_DataOut      (USBD_HandleTypeDef *pdev , uint8_t epnum);
static uint8_t* USBD_AUDIO_CDC_GetConfigDescriptor(uint16_t *length);
static uint8_t  *USBD_AUDIO_CDC_GetDeviceQualifierDesc (uint16_t *length);
#ifdef USB_OTG_HS_CORE
static uint8_t*  USBD_AUDIO_CDC_GetOtherConfigDescriptor( uint8_t speed , uint16_t *length);
#endif

/**
* @}
*/

/** @defgroup USBD_AUDIO_CDC_Private_Variables
* @{
*/
USBD_ClassTypeDef  USBD_AUDIO_CDC =
{
  USBD_AUDIO_CDC_Init,
  USBD_AUDIO_CDC_DeInit,
  USBD_AUDIO_CDC_Setup,
  NULL,
  USBD_AUDIO_CDC_EP0_RxReady,
  USBD_AUDIO_CDC_DataIn,
  USBD_AUDIO_CDC_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_AUDIO_CDC_GetConfigDescriptor,
  USBD_AUDIO_CDC_GetConfigDescriptor,
  USBD_AUDIO_CDC_GetConfigDescriptor,
  USBD_AUDIO_CDC_GetDeviceQualifierDesc,
};



__ALIGN_BEGIN static uint8_t USBD_AUDIO_CDC_CfgDesc[USB_HID_CDC_CONFIG_DESC_SIZ + 16] __ALIGN_END;

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};


/**
* @}
*/

/** @defgroup USBD_AUDIO_CDC_Private_Functions
* @{
*/



/**
* @brief  USBD_AUDIO_CDC_Init
*         Initialize the AUDIO and the CDC interface
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  USBD_AUDIO_CDC_Init (USBD_HandleTypeDef *pdev,
                                     uint8_t cfgidx)
{
  /* AUDIO initialization*/
  USBD_AUDIO_Init (pdev, cfgidx);
  
  /* CDC initialization */
  USBD_CDC_Init (pdev, cfgidx);
  
  return USBD_OK;
  
}

/**
* @brief  USBD_AUDIO_CDC_DeInit
*         Deinitialize the AUDIO and the CDC interface
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  USBD_AUDIO_CDC_DeInit (USBD_HandleTypeDef *pdev,
                                       uint8_t cfgidx)
{  
  /* AUDIO deinitialization */
  USBD_AUDIO_DeInit (pdev, cfgidx);
  
  /* CDC deinitialization */
  USBD_CDC_DeInit (pdev, cfgidx);
  
  return USBD_OK;
}

/**
* @brief  USBD_AUDIO_CDC_Setup
*         Handle the setup requests
* @param  pdev: instance
* @param  req: usb requests
* @retval status
*/
static uint8_t  USBD_AUDIO_CDC_Setup (USBD_HandleTypeDef *pdev,
                                      USBD_SetupReqTypedef *req)
{
  if (req->wIndex == CDC_COM_INTERFACE)
  {   
    return (USBD_CDC_Setup(pdev, req));
  }
  else
  {
    return (USBD_AUDIO_Setup(pdev, req));
  }  
}

/**
* @brief  USBD_AUDIO_CDC_GetConfigDescriptor
*         return configuration descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_AUDIO_CDC_GetConfigDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_AUDIO_CDC_CfgDesc);
  return USBD_AUDIO_CDC_CfgDesc;
}


/**
* @brief  USBD_AUDIO_CDC_EP0_RxReady
*         handle EP0 Rx Ready event
* @param  pdev: device instance
* @retval status
*/
static uint8_t  USBD_AUDIO_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{  
  if(USBD_CDC_EP0_RxReady(pdev) == USBD_OK)
  {
    return USBD_OK;
  }
  else
  {
    return (USBD_AUDIO_EP0_RxReady(pdev));   
  }  
}


/**
* @brief  USBD_AUDIO_DataIn
*         handle data IN Stage of the composite device
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
static uint8_t  USBD_AUDIO_CDC_DataIn       (USBD_HandleTypeDef *pdev , uint8_t epnum)
{  
  /*DataIN can be for CDC or AUDIO */
  if (epnum == (CDC_IN_EP & ~0x80) )
  {
    return (USBD_CDC_DataIn(pdev, epnum));
  }
  else
  {
    return (USBD_AUDIO_DataIn(pdev, epnum));
  }
}


/**
* @brief  USBD_AUDIO_CDC_DataOut
*         handle data OUT Stage of the composite device
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
static uint8_t  USBD_AUDIO_CDC_DataOut      (USBD_HandleTypeDef *pdev , uint8_t epnum)
{  
    /*DataIN can be for CDC only */
  return (USBD_CDC_DataOut(pdev, epnum));  
}

/**
* @brief  Configures the microphone descriptor on the base of the frequency 
*         and channels number informations. These parameters will be used to
*         init the audio engine, trough the USB interface functions.
* @param  samplingFrequency: sampling frequency
* @param  Channels: number of channels
* @retval status
*/
void USBD_AUDIO_CDC_Init_Microphone_Descriptor(uint32_t samplingFrequency, uint8_t Channels)
{
  uint16_t index;
  uint16_t AUDIO_CONTROLS;
  
  /* 9 */
  USBD_AUDIO_CDC_CfgDesc[0] = 0x09;  /* bLength */
  USBD_AUDIO_CDC_CfgDesc[1] = USB_DESC_TYPE_CONFIGURATION;      /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[2] = ((USB_HID_CDC_CONFIG_DESC_SIZ + Channels) & 0xff);         /* wTotalLength */
  USBD_AUDIO_CDC_CfgDesc[3] = ((USB_HID_CDC_CONFIG_DESC_SIZ + Channels) >> 8);
  USBD_AUDIO_CDC_CfgDesc[4] = 0x04;                                 /* bNumInterfaces */
  USBD_AUDIO_CDC_CfgDesc[5] = 0x01;                                 /* bConfigurationValue */
  USBD_AUDIO_CDC_CfgDesc[6] = 0x00;                                 /* iConfiguration */
  USBD_AUDIO_CDC_CfgDesc[7] = 0xE0;                                   /* bmAttributes  BUS Powered*/
  USBD_AUDIO_CDC_CfgDesc[8] = 0x32;                                 /* bMaxPower = 100 mA*/  
  index = 9;
  /*IAD*/ /* 8 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x08;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x0B;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO;
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_SUBCLASS_AUDIOCONTROL;
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_PROTOCOL_UNDEFINED;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  
  /* USB Microphone Standard interface descriptor */
  /* 9 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x09;            /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE;       /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                /* bInterfaceNumber */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                              /* bAlternateSetting */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                /* bNumEndpoints */
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO;              /* bInterfaceClass */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_SUBCLASS_AUDIOCONTROL;         /* bInterfaceSubClass */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_PROTOCOL_UNDEFINED;            /* bInterfaceProtocol */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                /* iInterface */
  
  /* USB Microphone Class-specific AC Interface Descriptor */
  /* 9 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x09;           /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;       /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_CONTROL_HEADER;              /* bDescriptorSubtype */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;       /* 1.00 */             /* bcdADC */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x26 + Channels;       /*check*/         /* wTotalLength = 37+AUDIO_CHANNELS_IN*/
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bInCollection */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* baInterfaceNr */
  
  /* USB Microphone Input Terminal Descriptor */
  /* 12 */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_INPUT_TERMINAL_DESC_SIZE;       /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;      /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_CONTROL_INPUT_TERMINAL;         /* bDescriptorSubtype */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bTerminalID */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* wTerminalType AUDIO_TERMINAL_USB_MICROPHONE   0x0201 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* bAssocTerminal */
  USBD_AUDIO_CDC_CfgDesc[index++] = Channels;                       /* bNrChannels */
  
  if(Channels != 2)
  {
    USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;  /* wChannelConfig 0x0000  Mono */
    USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  }
  else
  {
    USBD_AUDIO_CDC_CfgDesc[index++] = 0x03;      /* wChannelConfig 0x0003  Stereo */
    USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  }
  
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* iChannelNames */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* iTerminal */
  
  /* USB Microphone Audio Feature Unit Descriptor */
  /* 8 + Channel */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x07 + Channels + 1;            /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;      /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_CONTROL_FEATURE_UNIT;           /* bDescriptorSubtype */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;                                 /* bUnitID */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bSourceID */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bControlSize */
    if(Channels == 1)
  {
    AUDIO_CONTROLS = (0x02);
    USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_CONTROLS;
    USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  }
  else
  {
    AUDIO_CONTROLS = (0x02);    
    USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
    USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_CONTROLS;
    USBD_AUDIO_CDC_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }  
  if(Channels > 2)
  {
    USBD_AUDIO_CDC_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }  
  if(Channels > 3)
  {
    USBD_AUDIO_CDC_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }  
  if(Channels > 4)
  {
    USBD_AUDIO_CDC_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }  
  if(Channels > 5)
  {
    USBD_AUDIO_CDC_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }  
  if(Channels > 6)
  {
    USBD_AUDIO_CDC_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }  
  if(Channels > 7)
  {
    USBD_AUDIO_CDC_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }  
  USBD_AUDIO_CDC_CfgDesc[index] = 0x00;  /* iTerminal */
  index++;
  /*USB Microphone Output Terminal Descriptor */
  /* 9 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x09;      /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;      /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_CONTROL_OUTPUT_TERMINAL;        /* bDescriptorSubtype */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x03;                                 /* bTerminalID */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING 0x0101*/
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  
  /* USB Microphone Standard AS Interface Descriptor - Audio Streaming Zero Bandwith  Interface 1, Alternate Setting 0     */
  /* 9 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x09;  /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE;        /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bInterfaceNumber */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* bAlternateSetting */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* bNumEndpoints */
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO;               /* bInterfaceClass */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_SUBCLASS_AUDIOSTREAMING;        /* bInterfaceSubClass */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_PROTOCOL_UNDEFINED;             /* bInterfaceProtocol */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  
  /* USB Microphone Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
  /* 9 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x09;  /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE;        /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bInterfaceNumber */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bAlternateSetting */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bNumEndpoints */
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO;               /* bInterfaceClass */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_SUBCLASS_AUDIOSTREAMING;        /* bInterfaceSubClass */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_PROTOCOL_UNDEFINED;             /* bInterfaceProtocol */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* iInterface */
  
  /* USB Microphone Audio Streaming Interface Descriptor */
  /* 7 */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_STREAMING_INTERFACE_DESC_SIZE;  /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;      /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_STREAMING_GENERAL;              /* bDescriptorSubtype */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x03;                                 /* bTerminalLink */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bDelay */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  //  // ---------- ok ---------------
  /* USB Microphone Audio Type I Format Interface Descriptor */
  /* 11 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x0B;                                 /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;      /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_STREAMING_FORMAT_TYPE;          /* bDescriptorSubtype */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_FORMAT_TYPE_I;                  /* bFormatType */
  USBD_AUDIO_CDC_CfgDesc[index++] = Channels;                      /* bNrChannels */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;                                 /* bSubFrameSize */
  USBD_AUDIO_CDC_CfgDesc[index++] = 16;                                   /* bBitResolution */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bSamFreqType */
  USBD_AUDIO_CDC_CfgDesc[index++] = samplingFrequency & 0xff;                 /* tSamFreq 8000 = 0x1F40 */
  USBD_AUDIO_CDC_CfgDesc[index++] = (samplingFrequency >> 8) & 0xff;
  USBD_AUDIO_CDC_CfgDesc[index++] = samplingFrequency >> 16;
  
  /* Endpoint 1 - Standard Descriptor */
  /* 9 */
  USBD_AUDIO_CDC_CfgDesc[index++] =  AUDIO_STANDARD_ENDPOINT_DESC_SIZE;    /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x05;         /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_IN_EP;                                 /* bEndpointAddress 1 in endpoint*/
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x05;//USB_ENDPOINT_TYPE_ISOCHRONOUS;        /* bmAttributes */
  USBD_AUDIO_CDC_CfgDesc[index++] = ((samplingFrequency / 1000 + 2) * Channels * 2) & 0xFF; /* wMaxPacketSize */
  USBD_AUDIO_CDC_CfgDesc[index++] = ((samplingFrequency / 1000 + 2) * Channels * 2) >> 8;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;                                 /* bInterval */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* bRefresh */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* bSynchAddress */
  
  /* Endpoint - Audio Streaming Descriptor*/
  /* 7 */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_STREAMING_ENDPOINT_DESC_SIZE;   /* bLength */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_ENDPOINT_DESCRIPTOR_TYPE;       /* bDescriptorType */
  USBD_AUDIO_CDC_CfgDesc[index++] = AUDIO_ENDPOINT_GENERAL;               /* bDescriptor */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* bmAttributes */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* bLockDelayUnits */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;                                 /* wLockDelay */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  
  /*IAD*/
  /* 8 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x08;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x0B;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;//0x02;/*/*/
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  
   /*Interface Descriptor */
  /* 9 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x09;
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  
  /*Header Functional Descriptor*/
  /* 5 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x05;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x024;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x10;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;
  
  /*Call Management Functional Descriptor*/
  /* 5 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x05;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x24;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x01;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x03;
  
  /*ACM Functional Descriptor*/
  /* 4 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x04;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x24;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  
  /*Union Functional Descriptor*/
  /* 5 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x05;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x24;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x06;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x03;
  
  /*Endpoint 2 Descriptor*/
  /* 7 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x07;
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_DESC_TYPE_ENDPOINT;
  USBD_AUDIO_CDC_CfgDesc[index++] = CDC_CMD_EP;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x03;
  USBD_AUDIO_CDC_CfgDesc[index++] = LOBYTE(CDC_CMD_PACKET_SIZE);
  USBD_AUDIO_CDC_CfgDesc[index++] = HIBYTE(CDC_CMD_PACKET_SIZE);
  USBD_AUDIO_CDC_CfgDesc[index++] = 0xFF;
  
  /*Data class interface descriptor*/
  /* 9 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x09;
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x03;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x0A;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  
  /*Endpoint OUT Descriptor*/
  /* 7 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x07;
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_DESC_TYPE_ENDPOINT;
  USBD_AUDIO_CDC_CfgDesc[index++] = CDC_OUT_EP;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x40;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  
  /*Endpoint IN Descriptor*/
  /* 7 */
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x07;
  USBD_AUDIO_CDC_CfgDesc[index++] = USB_DESC_TYPE_ENDPOINT;
  USBD_AUDIO_CDC_CfgDesc[index++] = CDC_IN_EP;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x40;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CDC_CfgDesc[index++] = 0x00;
  
  haudioInstance.paketDimension = (samplingFrequency/1000*Channels*2);
  haudioInstance.frequency=samplingFrequency;
  haudioInstance.buffer_length = haudioInstance.paketDimension * AUDIO_IN_PACKET_NUM;
  haudioInstance.channels=Channels;  
  haudioInstance.upper_treshold = 5;
  haudioInstance.lower_treshold = 2;
  haudioInstance.state = STATE_USB_WAITING_FOR_INIT;
  haudioInstance.wr_ptr = 3 * haudioInstance.paketDimension;
  haudioInstance.rd_ptr = 0;  
  haudioInstance.dataAmount=0;
  haudioInstance.buffer = 0;  
}

/**
* @brief  USBD_AUDIO_Init
*         Initialize the AUDIO interface
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{
  if(haudioInstance.state!=STATE_USB_WAITING_FOR_INIT)
  {
    return USBD_FAIL; 
  }
  
  USBD_AUDIO_HandleTypeDef   *haudio;
  pdev->pClassData = &haudioInstance;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;
  uint16_t packet_dim = haudio->paketDimension;
  uint16_t wr_rd_offset = (AUDIO_IN_PACKET_NUM/2) * haudio->dataAmount / haudio->paketDimension;
  haudio->wr_ptr=wr_rd_offset * packet_dim;
  haudio->rd_ptr = 0;
  haudio->timeout = 0;
  
  ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init_Audio(haudio->frequency,0,haudio->channels);
  
  USBD_LL_OpenEP(pdev,
                 AUDIO_IN_EP,
                 USBD_EP_TYPE_ISOC,
                 AUDIO_IN_PACKET);
  
  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  
  
  USBD_LL_Transmit(pdev, AUDIO_IN_EP,
                   IsocInBuffDummy,                        
                   packet_dim);      
  
  haudio->state=STATE_USB_IDLE;
  return USBD_OK;
}

/**
* @brief  USBD_AUDIO_DeInit
*         DeInitialize the AUDIO layer
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev,
                                   uint8_t cfgidx)
{
  
  /* Close EP IN */
  USBD_LL_CloseEP(pdev,
                  AUDIO_IN_EP);
  
  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit(0);
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  
  return USBD_OK;
}

/**
* @brief  USBD_AUDIO_Setup
*         Handle the AUDIO specific requests
* @param  pdev: instance
* @param  req: usb requests
* @retval status
*/
static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev,
                                  USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  uint16_t len = 0;
  uint8_t *pbuf;
  uint8_t ret = USBD_OK;
  haudio=&haudioInstance;  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* AUDIO Class Requests -------------------------------*/
  case USB_REQ_TYPE_CLASS :
    
    switch (req->bRequest)
    {
    case AUDIO_REQ_GET_CUR:
      AUDIO_REQ_GetCurrent(pdev, req);
      break;
      
    case AUDIO_REQ_SET_CUR:
      AUDIO_REQ_SetCurrent(pdev, req);
      break;
      
    case AUDIO_REQ_GET_MIN:
      AUDIO_REQ_GetMinimum(pdev, req);
      break;
      
    case AUDIO_REQ_GET_MAX:
      AUDIO_REQ_GetMaximum(pdev, req);
      break;
      
    case AUDIO_REQ_GET_RES:
      AUDIO_REQ_GetResolution(pdev, req);
      break;
      
    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
    }
    break;
    
    
    /* Standard Requests -------------------------------*/
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR:
      if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
      {
        
        pbuf = USBD_AUDIO_CDC_CfgDesc + 18;
        len = MIN(USB_AUDIO_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev,
                        pbuf,
                        len);
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)haudio->alt_setting,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < USBD_MAX_NUM_INTERFACES)
      {
        haudio->alt_setting = (uint8_t)(req->wValue);
      }
      else
      {
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req);
      }
      break;
    }
  }
  return ret;
}


/**
* @brief  USBD_AUDIO_DataIn
*         handle data IN Stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev,
                                   uint8_t epnum)
{ 
  
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio=&haudioInstance;
  uint32_t length_usb_pck;
  uint16_t app;
  uint16_t IsocInWr_app = haudio->wr_ptr;
  uint16_t true_dim = haudio->buffer_length;
  uint16_t packet_dim = haudio->paketDimension;
  uint16_t channels = haudio->channels;
  length_usb_pck = packet_dim;  
  haudio->timeout=0;
  if (epnum == (AUDIO_IN_EP & 0x7F))
  {    
    if (haudio->state == STATE_USB_IDLE) 
    {
      haudio->state=STATE_USB_REQUESTS_STARTED;
      ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Record();      
    }    
    if (haudio->state == STATE_USB_BUFFER_WRITE_STARTED)   
    {      
      haudio->rd_ptr = haudio->rd_ptr % (true_dim);              
      if(IsocInWr_app<haudio->rd_ptr){
        app = ((true_dim) - haudio->rd_ptr) +  IsocInWr_app;
      }else{
        app = IsocInWr_app - haudio->rd_ptr;
      }    

      if(app >= (packet_dim*haudio->upper_treshold)){       
        length_usb_pck += channels*2;
      }else if(app <= (packet_dim*haudio->lower_treshold)){
        length_usb_pck -= channels*2;
      }     
      
      USBD_LL_Transmit (pdev,AUDIO_IN_EP,
                        (uint8_t*)(&haudio->buffer[haudio->rd_ptr]),
                        length_usb_pck);   
      
      haudio->rd_ptr += length_usb_pck;      
      
      if(app < haudio->buffer_length/10)
      {
        ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Stop();
        haudio->state = STATE_USB_IDLE; 
        haudio->timeout=0;
        memset(haudio->buffer,0,(haudio->buffer_length + haudio->dataAmount));
      }       
    }
    else 
    {      
      USBD_LL_Transmit (pdev,AUDIO_IN_EP,
                        IsocInBuffDummy,
                        length_usb_pck);      
    }    
  }
  return USBD_OK;
}

/**
* @brief  USBD_AUDIO_EP0_RxReady
*         handle EP0 Rx Ready event
* @param  pdev: device instance
* @retval status
*/
static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio=&haudioInstance;
  if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
  {
    /* In this driver, to simplify code, only SET_CUR request is managed */    
    if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
    {
      ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->AudioCtl(haudio->control.cmd,
                                                              (uint8_t *)haudio->control.data,
                                                              haudio->control.len);
        haudio->control.cmd = 0;
        haudio->control.len = 0;
        haudio->control.unit = 0;
        haudio->control.data[0] = 0;
        haudio->control.data[0] = 0;
    }
  }
  
  
  return USBD_OK;
}


/**
* @brief  USBD_AUDIO_Data_Transfer
*         Fills the USB internal buffer with audio data from user
* @param pdev: device instance
* @param audioData: audio data to be sent via USB
* @param dataAmount: number of PCM samples to be copyed
* @note Depending on the calling frequency, a coherent amount of samples must be passed to 
*       the function. E.g.: assuming a Sampling frequency of 16 KHz and 1 channel, 
*       you can pass 16 PCM samples if the function is called each millisecond, 
*       32 samples if called every 2 milliseconds and so on. 
* @retval status
*/
uint8_t  USBD_AUDIO_Data_Transfer(USBD_HandleTypeDef *pdev, int16_t * audioData, uint16_t PCMSamples)
{
  
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio=&haudioInstance;
  
  if(haudioInstance.state==STATE_USB_WAITING_FOR_INIT){    
    return USBD_BUSY;    
  }  
  uint16_t dataAmount = PCMSamples * 2; /*Bytes*/
  uint16_t true_dim = haudio->buffer_length;
  uint16_t current_data_Amount = haudio->dataAmount;
  uint16_t packet_dim = haudio->paketDimension;
  
  if(haudio->state==STATE_USB_REQUESTS_STARTED  || current_data_Amount!=dataAmount){   
    
    /*USB parameters definition, based on the amount of data passed*/
    haudio->dataAmount=dataAmount;                  
    uint16_t wr_rd_offset = (AUDIO_IN_PACKET_NUM/2) * dataAmount / packet_dim; 
    haudio->wr_ptr=wr_rd_offset * packet_dim;
    haudio->rd_ptr = 0;
    haudio->upper_treshold = wr_rd_offset + 2;
    haudio->lower_treshold = wr_rd_offset - 2;
    haudio->buffer_length = (packet_dim * (dataAmount / packet_dim) * AUDIO_IN_PACKET_NUM);
    
    /*Memory allocation for data buffer, depending (also) on data amount passed to the transfer function*/
    if(haudio->buffer != NULL)
    {
      USBD_free(haudio->buffer);      
    }
    haudio->buffer = USBD_malloc(haudio->buffer_length + haudio->dataAmount);
    if(haudio->buffer == NULL)
    {
      return USBD_FAIL;       
    }
    memset(haudio->buffer,0,(haudio->buffer_length + haudio->dataAmount));
    haudio->state=STATE_USB_BUFFER_WRITE_STARTED;
    
    
  }else if(haudio->state==STATE_USB_BUFFER_WRITE_STARTED){
    if(haudio->timeout++==TIMEOUT_VALUE){
      haudio->state=STATE_USB_IDLE;
      ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Stop();   
      haudio->timeout=0;
    }
    memcpy((uint8_t * )&haudio->buffer[haudio->wr_ptr], (uint8_t *)(audioData), dataAmount);    
    haudio->wr_ptr += dataAmount;
    haudio->wr_ptr = haudio->wr_ptr % (true_dim);    
    if((haudio->wr_ptr-dataAmount) == 0){
      memcpy((uint8_t *)(((uint8_t *)haudio->buffer)+true_dim),(uint8_t *)haudio->buffer, dataAmount);
    }
  }
  return USBD_OK;  
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_AUDIO_CDC_GetDeviceQualifierDesc (uint16_t *length)  
{
  *length = sizeof (USBD_AUDIO_CDC_DeviceQualifierDesc);
  return USBD_AUDIO_CDC_DeviceQualifierDesc;
}

/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                        USBD_AUDIO_CDC_ItfTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pUserData[0] = fops;
  }
  return 0;
}

/**
* @brief  AUDIO_REQ_GetMaximum
*         Handles the VOL_MAX Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_GetMaximum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio=&haudioInstance;  
  ((int16_t*)(haudio->control.data))[0] = (VOL_MAX);  
  USBD_CtlSendData (pdev,
                    haudio->control.data,
                    req->wLength);                    
}

/**
* @brief  AUDIO_REQ_GetMinimum
*         Handles the VOL_MIN Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_GetMinimum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio=&haudioInstance;

  ((int16_t*)(haudio->control.data))[0] = (VOL_MIN);
  /* Send the current mute state */
  USBD_CtlSendData (pdev,
                    haudio->control.data,
                    req->wLength);
}

/**
* @brief  AUDIO_Req_GetResolution
*         Handles the VOL_RES Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_GetResolution(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio=&haudioInstance;

  ((int16_t*)(haudio->control.data))[0] = (VOL_RES);
  
  USBD_CtlSendData (pdev,
                    haudio->control.data,
                    req->wLength);
}

/**
* @brief  AUDIO_Req_GetCurrent
*         Handles the GET_CUR Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio=&haudioInstance;
  ((int16_t*)(haudio->control.data))[0] = VOL_CUR;  
  USBD_CtlSendData (pdev,
                    haudio->control.data,
                    req->wLength);  
}

/**
* @brief  AUDIO_Req_SetCurrent
*         Handles the SET_CUR Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio=&haudioInstance;
  if (req->wLength)
  {    
    haudio->control.cmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    haudio->control.len = req->wLength;          /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */    
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev,
                       (uint8_t *)haudio->control.data,
                       req->wLength);
  }
}

/**
* @brief  USBD_CDC_Init
*         Initilaize the CDC interface
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_CDC_HandleTypeDef   *hcdc;  
  if(pdev->dev_speed == USBD_SPEED_HIGH  )
  {
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                   CDC_IN_EP,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_HS_IN_PACKET_SIZE);    
    /* Open EP OUT */
    USBD_LL_OpenEP(pdev,
                   CDC_OUT_EP,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_HS_OUT_PACKET_SIZE);    
  }
  else
  {
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                   CDC_IN_EP,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_IN_PACKET_SIZE);    
    /* Open EP OUT */
    USBD_LL_OpenEP(pdev,
                   CDC_OUT_EP,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_OUT_PACKET_SIZE);
  }
  /* Open Command IN EP */
  USBD_LL_OpenEP(pdev,
                 CDC_CMD_EP,
                 USBD_EP_TYPE_INTR,
                 CDC_CMD_PACKET_SIZE);
  
  pdev->pClassData = USBD_malloc(sizeof (USBD_CDC_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    ret = 1;
  }
  else
  {
    hcdc = pdev->pClassData;
    
    /* Init  physical Interface components */
    ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init_CDC();
    
    /* Init Xfer states */
    hcdc->TxState = 0;
    hcdc->RxState = 0;
    
    if(pdev->dev_speed == USBD_SPEED_HIGH  )
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP,
                             hcdc->RxBuffer,
                             CDC_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP,
                             hcdc->RxBuffer,
                             CDC_DATA_FS_OUT_PACKET_SIZE);
    }
    
    
  }
  return ret;
}

/**
* @brief  USBD_CDC_DeInit
*         DeInitialize the CDC layer
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{
  uint8_t ret = 0;
  
  /* Open EP IN */
  USBD_LL_CloseEP(pdev,
                  CDC_IN_EP);
  
  /* Open EP OUT */
  USBD_LL_CloseEP(pdev,
                  CDC_OUT_EP);
  
  /* Open Command IN EP */
  USBD_LL_CloseEP(pdev,
                  CDC_CMD_EP);
  
  
  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit(0);
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  
  return ret;
}

/**
* @brief  USBD_CDC_Setup
*         Handle the CDC specific requests
* @param  pdev: instance
* @param  req: usb requests
* @retval status
*/
static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  USBD_CDC_HandleTypeDef   *hcdc = pdev->pClassData;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength)
    {
      if (req->bmRequest & 0x80)
      {
        ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->CDCCtl(req->bRequest,
                                                                (uint8_t *)hcdc->data,
                                                                req->wLength);
          USBD_CtlSendData (pdev,
                            (uint8_t *)hcdc->data,
                            req->wLength);
      }
      else
      {
        hcdc->CmdOpCode = req->bRequest;
        hcdc->CmdLength = req->wLength;
        
        USBD_CtlPrepareRx (pdev,
                           (uint8_t *)hcdc->data,
                           req->wLength);
      }
      
    }
    else
    {
      ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->CDCCtl(req->bRequest,
                                                              NULL,
                                                              0);
    }
    break;
    
  default:
    break;
  }
  return USBD_OK;
}

/**
* @brief  USBD_CDC_DataIn
*         Data sent on non-control IN endpoint
* @param  pdev: device instance
* @param  epnum: endpoint number
* @retval status
*/
static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef   *hcdc = pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    
    hcdc->TxState = 0;
    
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
* @brief  USBD_CDC_DataOut
*         Data received on non-control Out endpoint
* @param  pdev: device instance
* @param  epnum: endpoint number
* @retval status
*/
static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef   *hcdc = pdev->pClassData;  
  /* Get the received data length */
  hcdc->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);  
  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */
  if(pdev->pClassData != NULL)
  {
    ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Receive(hcdc->RxBuffer, &hcdc->RxLength);    
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
* @brief  USBD_CDC_DataOut
*         Data received on non-control Out endpoint
* @param  pdev: device instance
* @param  epnum: endpoint number
* @retval status
*/
static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef   *hcdc = pdev->pClassData;
  
  if((pdev->pUserData[pdev->classId] != NULL) && (hcdc->CmdOpCode != 0xFF))
  {
    
    
    ((USBD_AUDIO_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->CDCCtl(hcdc->CmdOpCode,
                                                            (uint8_t *)hcdc->data,
                                                            hcdc->CmdLength);
      hcdc->CmdOpCode = 0xFF;
      
      return USBD_OK;
  }
  return USBD_FAIL;
}

/**
* @brief  USBD_AUDIO_CDC_RegisterInterface
* @param  pdev: device instance
* @param  fops: CD  Interface callback
* @retval status
*/
uint8_t  USBD_AUDIO_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                            USBD_AUDIO_CDC_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  
  if(fops != NULL)
  {
    pdev->pUserData[0] = fops;
    ret = USBD_OK;
  }
  
  return ret;
}

/**
* @brief  USBD_CDC_SetTxBuffer
* @param  pdev: device instance
* @param  pbuff: Tx Buffer
* @retval status
*/
uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length)
{
  USBD_CDC_HandleTypeDef   *hcdc = pdev->pClassData;
  
  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;
  
  return USBD_OK;
}

/**
* @brief  USBD_CDC_SetRxBuffer
* @param  pdev: device instance
* @param  pbuff: Rx Buffer
* @retval status
*/
uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff)
{
  USBD_CDC_HandleTypeDef   *hcdc = pdev->pClassData;
  
  hcdc->RxBuffer = pbuff;
  
  return USBD_OK;
}

/**
* @brief  USBD_CDC_DataOut
*         Data received on non-control Out endpoint
* @param  pdev: device instance
* @param  epnum: endpoint number
* @retval status
*/
uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef   *hcdc = pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    if(hcdc->TxState == 0)
    {
      
      /* Transmit next packet */
      USBD_LL_Transmit(pdev,
                       CDC_IN_EP,
                       hcdc->TxBuffer,
                       hcdc->TxLength);
      
      /* Tx Transfer in progress */
      hcdc->TxState = 1;
      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}


/**
* @brief  USBD_CDC_ReceivePacket
*         prepare OUT Endpoint for reception
* @param  pdev: device instance
* @retval status
*/
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef   *hcdc = pdev->pClassData;
  
  /* Suspend or Resume USB Out process */
  if(pdev->pClassData != NULL)
  {
    if(pdev->dev_speed == USBD_SPEED_HIGH  )
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP,
                             hcdc->RxBuffer,
                             CDC_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP,
                             hcdc->RxBuffer,
                             CDC_DATA_FS_OUT_PACKET_SIZE);
    }
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
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

