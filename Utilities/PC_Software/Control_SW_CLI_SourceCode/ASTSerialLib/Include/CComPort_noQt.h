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
  
#ifndef CCOMPORT_NOQT_H
#define CCOMPORT_NOQT_H

#include "CCronometer.h"

#ifdef _WIN32
#include "Shlwapi.h"
#endif

#ifdef __linux__
#include "termios.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#endif


class CComPort
{
public:
    static const int FBufferMaxLen=2048;

private:

    int FOpened;
    int FPortNumber;
    int FPortSpeed;
    int FPortSize;
    int FPortParity;
    int FPortstopBits;
    unsigned char FEndOfFrame;
    int FBufferLen;
    unsigned char FBuffer[FBufferMaxLen];
#ifdef __linux__
    struct termios newtio,oldtio;
    int fd;
#endif
#ifdef _WIN32
    HANDLE hCom;
#endif



private:
#ifdef _WIN32
    void ctow(WCHAR* Dest, const CHAR* Source);
#endif
    int Read(unsigned char *Buffer, int NumToRead, int *NumReceived);

public:
    CComPort(void);
    ~CComPort(void);

    int Open(int PortSpeed = 9600);
    int Opened();
#ifdef _WIN32
    int IsRS232();
#endif
    int Close();
    void SetPortNumber(int Value);
    void SetPortSpeed(int Value);
    void SetEndOfFrame(unsigned char  Value);

    int Write(unsigned char *Buffer, int NumToWrite, int *NumWritten);
    int ReadWait(unsigned char *Buffer, int NumToRead, int *NumReceived, double MaxWait);
    int ReadFrame(unsigned char *Buffer, int MaxDim, int *FrameAvailable, double MaxWait);
};

#endif

