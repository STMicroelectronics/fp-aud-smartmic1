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
  
#ifndef CCOMPORT_H
#define CCOMPORT_H

#include <QObject>
#include <QSerialPort>


class CComPort : public QObject
{
    Q_OBJECT

public:
    static const int FBufferMaxLen=2048;

private:
    QSerialPort *serialPort;
    unsigned char FEndOfFrame;
    int FBufferLen;
    unsigned char FBuffer[FBufferMaxLen];
    quint32 BaudRate;

public:
    explicit CComPort(QObject *parent = 0);
    ~CComPort();

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
    int ReadFrame(unsigned char *Buffer, int MaxDim, int *FrameAvailable, double MaxWait);

};

#endif

