/**
  ******************************************************************************
  * @file    
  * @author  SRA
  * @brief 
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
  
#ifndef KBHITh
#define KBHITh

#ifdef __cplusplus
extern "C" {
#endif

void init_keyboard(void);
void close_keyboard(void);
int kbhit(void);
int readch(void);

#ifdef __cplusplus
}
#endif

#endif 
