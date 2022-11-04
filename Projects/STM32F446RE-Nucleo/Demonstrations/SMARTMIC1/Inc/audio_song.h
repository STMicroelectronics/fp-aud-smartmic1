/**
  ******************************************************************************
  * @file    audio_song.h
  * @author  SRA
  * 
  * 
  * @brief   Header for the definition of the "song" type.
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
#ifndef __AUDIO_SONGS_H
#define __AUDIO_SONGS_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
  int16_t *data;
  uint32_t length;
  uint32_t in_ptr;
  uint32_t out_ptr;
} Song_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */



#endif /* __AUDIO_SONGS_H */


