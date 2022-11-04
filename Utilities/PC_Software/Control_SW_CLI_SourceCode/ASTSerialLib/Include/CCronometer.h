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
  
#ifndef CRONOMETER_H
#define CRONOMETER_H

#ifdef __linux__
#include <sys/time.h>
#include <stdio.h>
#include <time.h>
#endif


class CCronometer {
private:
        #ifdef _WIN32
        long long Frequency;
        long long StartTime;
        long long LastTime;
        #endif
        #ifdef __linux__
        struct timeval tv_StartTime,tv_LastTime;
        #endif

public:
        CCronometer(void);
        void Reset(void);
        double CurrentTime(void);
        double PartialTime(void);
};

#endif
