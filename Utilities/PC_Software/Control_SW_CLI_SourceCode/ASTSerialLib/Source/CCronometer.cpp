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
  
#include "CCronometer.h"

#ifdef _WIN32
extern "C" {
#include <windows.h>
}
#else
#include <unistd.h>
#endif



CCronometer::CCronometer(void)
{
    Reset();
}

/**
 * Reset the cronometer.
 *
 **/
void CCronometer::Reset(void)
{
#ifdef _WIN32
    LARGE_INTEGER li;
    QueryPerformanceFrequency(&li);
    Frequency = li.QuadPart;
    QueryPerformanceCounter(&li);
    StartTime = li.QuadPart;
    LastTime = StartTime;
#endif
#ifdef __linux__
    gettimeofday(&tv_StartTime, NULL);
    tv_LastTime.tv_sec=tv_StartTime.tv_sec;
    tv_LastTime.tv_usec=tv_StartTime.tv_usec;
#endif
}


/**
 * Compute current time.
 *
 * \return Returns current time.
 **/
double CCronometer::CurrentTime(void)
{
    double App;
#ifdef _WIN32
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    App = (double)(li.QuadPart - StartTime);
    App = App / Frequency;
#endif

#ifdef __linux__
    struct timeval tv_App;

    gettimeofday(&tv_App, NULL);
    App = (double)(tv_App.tv_sec-tv_StartTime.tv_sec) + (double)(tv_App.tv_usec-tv_StartTime.tv_usec)/(double)1000000;
#endif

    return App;
}

/**
 * Compute the partial time with respect to the last time requested.
 * \return Returns current time.
 **/
double CCronometer::PartialTime(void)
{
    double App;

#ifdef _WIN32
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    App = (double)(li.QuadPart - LastTime);
    App /= Frequency;
    LastTime = li.QuadPart;
#endif

#ifdef __linux__
    struct timeval tv_App;

    gettimeofday(&tv_App, NULL);

    App = (double)(tv_App.tv_sec-tv_LastTime.tv_sec) + (double)(tv_App.tv_usec-tv_LastTime.tv_usec)/(double)1000000;

    gettimeofday(&tv_LastTime, NULL);
#endif

    return App;
}


