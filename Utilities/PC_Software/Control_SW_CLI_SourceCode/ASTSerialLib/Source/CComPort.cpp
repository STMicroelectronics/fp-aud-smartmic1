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
  
#include "CComPort.h"
#include "CCronometer.h"
#include <QDebug>
#include <QSerialPortInfo>

CComPort::CComPort(QObject *parent) :
    QObject(parent)
{
    serialPort = new QSerialPort();

    serialPort->setReadBufferSize(FBufferMaxLen);
    BaudRate = QSerialPort::UnknownBaud;

    FEndOfFrame='\0';
    FBufferLen=0;
}

CComPort::~CComPort()
{
    delete serialPort;
}

/**
 * Check if the COM port is a RS232
 *
 * \return Returns -1 if an error occurs, 1 if the COM Port type is RS232 otherwise 0.
**/
#ifdef _WIN32
int CComPort::IsRS232()
{
    QSerialPortInfo PortInfo(*serialPort);

    return (PortInfo.hasProductIdentifier()?1:0);
}
#endif

/**
 * Close the communication on the serial bus.
 *
 * \return Returns 1 if the port is closed succesfully, 0 instead.
**/
int CComPort::Close()
{
    if (!serialPort->isOpen()) return 0;
    serialPort->close();
    return 1;
}

/**
 * Set com port number.
 * \param Value Set port number x, referring to /dev/ttyACMx.
 * 
**/
void CComPort::SetPortNumber(int Value)
{
#ifdef _WIN32
    serialPort->setPortName("COM" + QString::number(Value));
#endif
#ifdef __linux__
    serialPort->setPortName("/dev/ttyACM" + QString::number(Value));
#endif
}

/**
 * Set com port speed.
 * \param Value Port speed in bauds.
 * 
**/
void CComPort::SetPortSpeed(int Value)
{
    BaudRate = Value;
    if(serialPort->isOpen()) {
        serialPort->setBaudRate(BaudRate);
    }
}

/**
 * Set end of frame character.
 * \param Value Identifier for end of frame.
 * 
**/
void CComPort::SetEndOfFrame(unsigned char  Value)
{
    FEndOfFrame=Value;
}

/**
 * Transmit the content of the buffer.  Writes NumToWrite bytes to the buffer pointed by *Buffer.
 * \param Buffer Array of data.
 * \param NumToWrite Length of bytes to be write.
 * \param NumWritten Length of bytes which have been actually written.
 * \return Returns 0 in case of errors, 1 instead.
**/
int CComPort::Write(unsigned char *Buffer, int NumToWrite, int *NumWritten)
{
    *NumWritten = serialPort->write((char*)Buffer, NumToWrite);

    if(*NumWritten == -1) {
        return 0;
    }
    return 1;
}

/**
 * Reads MaxDim characters, writing them to Buffer. The function exits when MaxDim bytes have been read, when
 * maxWait (seconds) expires, or when a eof character arrives.
 * \param Buffer Array of data which have been read.
 * \param MaxDim Length of bytes to be read.
 * \param FrameAvailable 1 in caso of Frame received,0 instead.
 * \param MaxWait  Expires after (seconds).
 * \return Returns 0 in case of errors, 1 instead.
**/
int CComPort::ReadFrame(unsigned char *Buffer, int MaxDim, int *FrameAvailable, double MaxWait)
{
    int i=0;
    *FrameAvailable=0;

    if (!Opened()) return 0;
    if (MaxDim>FBufferMaxLen) return 0;
    do {
        FBufferLen += serialPort->read((char*)&FBuffer[FBufferLen], FBufferMaxLen-FBufferLen);

        while (i<FBufferLen) {
            if ((FBuffer[i]==FEndOfFrame) || (i>=(MaxDim-1))) {
                if (i==0) { // Remove a single EOF
                    FBufferLen--;
                    memcpy((char *)FBuffer, (char *) &FBuffer[1], FBufferLen);
                } else { // Manage a real Msg
                    i++;

                    // Copy the Msg to the output buffer
                    memcpy((char *)Buffer, (char *) FBuffer, i);
                    *FrameAvailable = 1;

                    // Copy the remaining bytes to the beginning of the internal buffer
                    memcpy((char *)FBuffer, (char *) &FBuffer[i], FBufferLen-i);

                    FBufferLen-=i;
                    return 1;
                }
            } else {
                i++;
            }
        }
    } while (serialPort->waitForReadyRead(MaxWait*1000));
    return 1;
}

/**
 * Check if the port is Open.
 *
 * \return Returns 1 if the port is open, 0 instead.
**/
int CComPort::Opened()
{
    return serialPort->isOpen();
}

/**
 * Open the Com port.
 *
 * \param PortSpeed Speed of the port in bauds.
 * \return Returns 0 if errors occur, 1 instead.
**/
int CComPort::Open(int PortSpeed)
{
    BaudRate = PortSpeed;
    if(!serialPort->open(QIODevice::ReadWrite)) {
        // Error
        return 0;
    }

    serialPort->setBaudRate(BaudRate);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setDataErrorPolicy(QSerialPort::IgnorePolicy);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);

    return 1;
}

