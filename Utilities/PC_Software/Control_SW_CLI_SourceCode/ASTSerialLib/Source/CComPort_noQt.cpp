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
  
#include "CComPort_noQt.h"
#include <stdio.h>

#ifdef __linux__
#include <string.h>
long PARITYON;
long PARITY;
#endif

// ****************************************************************** //
//                      PRIVATE FUNCTIONS                             //
// ****************************************************************** //
#ifdef _WIN32
/**
 * Convert from CHAR to WCHAR.
 *
 * \param Source Source array of CHAR.
 * \param Dest  Destination array of WCHAR.
**/
void CComPort::ctow(WCHAR* Dest, const CHAR* Source)
{
    int i = 0;

    while(Source[i] != '\0')
    {
        Dest[i] = (WCHAR)Source[i];
        ++i;
    }
    Dest[i] = (WCHAR)Source[i];
}
#endif

/**
 * Read a buffer from serial bus.
 *
 * \param Buffer Array where the data will be stored.
 * \param NumToRead Number of expected byte.
 * \param NumReceived Number of received byte.
 * \return Returns 0 if errors occur, 1 instead.
**/
int CComPort::Read(unsigned char *Buffer, int NumToRead, int *NumReceived)
{

    if (!Opened()) return 0;
#ifdef _WIN32
    DWORD l;
    if (!ReadFile(hCom, Buffer, NumToRead, &l, 0)) return 0;
    *NumReceived = l;
#endif
#ifdef __linux__
    int ret;

    ret = read(fd, Buffer,NumToRead);
//    unsigned char b2[15];
//    for(int k=0;k<15;k++){
//        b2[k]=Buffer[k];
//    }

    if(ret==-1){
        *NumReceived = 0;
        return 0;
    }else{
        *NumReceived = ret;
    }
#endif
    return 1;
}



// ****************************************************************** //
//                       PUBLIC FUNCTIONS                             //
// ****************************************************************** //


CComPort::CComPort(void)
{
#ifdef _WIN32
    hCom = INVALID_HANDLE_VALUE;
    FPortNumber=1;
    FPortSpeed=9600;		  // set the baud rate es CBR_9600
    FPortSize=8;              // data size, Tx and Rx
    FPortParity=NOPARITY;        // no parity bit
    FPortstopBits=ONESTOPBIT;    // one stop bit
#endif
#ifdef __linux__
    fd = -1;
    FPortNumber = 0;
    FPortSpeed  = B9600;  // set the baud rate es CBR_9600

    PARITYON = 0;
    PARITY = 0;

    FPortSize     = CS8;
    FPortstopBits = 0; // CstopB;
    FPortParity   = 0; // PARENB | PARODD;
#endif
    FOpened = 0;
    FEndOfFrame='\0';
    FBufferLen=0;
}

CComPort::~CComPort(void)
{
    if (Opened()) {
        Close();
    }
}

/**
 * Check if the COM port is a RS232
 *
 * \return Returns -1 if an error occurs, 1 if the COM Port type is RS232 otherwise 0.
**/
#ifdef _WIN32
int CComPort::IsRS232()
{
    COMMPROP CommProp;

    if (!FOpened) return -1;

    GetCommProperties(hCom, &CommProp);
    if(CommProp.dwProvSubType==PST_RS232)
         return 1;
    return 0;
}
#endif


/**
 * Close the communication on the serial bus.
 *
 * \return Returns 1 if the port is closed succesfully, 0 instead.
**/
int CComPort::Close()
{
    if (!FOpened) return 0;
#ifdef _WIN32

    if(!CloseHandle(hCom)) {
        return 0;
    }
#endif
#ifdef __linux__

    /* restore the old port settings */
    tcsetattr(fd,TCSANOW,&oldtio);

    if(close(fd)!=0) {
        return 0; // error while trying to close
    }
#endif
    FOpened = 0; // close successfully
    return 1;
}


/**
 * Set com port number.
 * \param Value Set port number x, referring to /dev/ttyACMx.
 * 
**/
void CComPort::SetPortNumber(int Value)
{
    FPortNumber=Value;
}


/**
 * Set com port speed.
 * \param Value Port speed in bauds.
 * 
**/
void CComPort::SetPortSpeed(int Value)
{
#ifdef _WIN32
    FPortSpeed=Value;
#endif
#ifdef __linux__
    switch (Value)
    {
		case 460800:
			FPortSpeed  = B460800;
			break;
		case 500000:
			FPortSpeed  = B500000;
			break;
		case 576000:
			FPortSpeed  = B576000;
			break;
		case 921600:
			FPortSpeed  = B921600;
			break;
		case 1000000:
			FPortSpeed  = B1000000;
			break;
		case 1152000:
			FPortSpeed  = B1152000;
			break;
		case 1500000:
			FPortSpeed  = B1500000;
			break;
		case 2000000:
			FPortSpeed  = B2000000;
			break;
		case 2500000:
			FPortSpeed  = B2500000;
			break;
		case 3000000:
			FPortSpeed  = B3000000;
			break;
		case 3500000:
			FPortSpeed  = B3500000;
			break;
		case 4000000:
			FPortSpeed  = B4000000;
			break;
        case 230400:
			FPortSpeed  = B230400;
			break;
		case 115200:
			FPortSpeed  = B115200;
			break;
        case 57600:
        	FPortSpeed  = B57600;
        	break;
        case 38400:
        default:
        	FPortSpeed = B38400;
        	break;
        case 19200:
        	FPortSpeed  = B19200;
        	break;
        case 9600:
        	FPortSpeed  = B9600;
        	break;
        case 4800:
        	FPortSpeed  = B4800;
        	break;
        case 2400:
        	FPortSpeed  = B2400;
        	break;
        case 1800:
        	FPortSpeed  = B1800;
        	break;
        case 1200:
        	FPortSpeed  = B1200;
        	break;
        case 600:
        	FPortSpeed  = B600;
        	break;
        case 300:
        	FPortSpeed  = B300;
        	break;
        case 200:
        	FPortSpeed  = B200;
        	break;
        case 150:
        	FPortSpeed  = B150;
        	break;
        case 134:
        	FPortSpeed  = B134;
        	break;
        case 110:
        	FPortSpeed  = B110;
        	break;
        case 75:
        	FPortSpeed  = B75;
        	break;
        case 50:
        	FPortSpeed  = B50;
        	break;
    }  //end of switch baud_rate
#endif
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
    int ret;

#ifdef _WIN32
    DWORD Len;
    ret = WriteFile(hCom, Buffer, NumToWrite, &Len, 0);
    *NumWritten=Len;
#endif
#ifdef __linux__
    ret = write(fd, Buffer,NumToWrite);
    unsigned char b2[15];
    for(int k=0;k<15;k++){
        b2[k]=Buffer[k];
    }
    if(ret!=-1) {
        *NumWritten = ret;
        ret=1;  //success
    }
    else {
        ret=0;
        *NumWritten = 0;
    }
#endif
    return ret;
}


/**
 * Reads NumToRead characters, writing them to Buffer. The function exits when NumToRead bytes have been read or when
 *  maxWait (seconds) expires.
 * \param Buffer Array of data which have been read.
 * \param NumToRead Length of bytes to be read.
 * \param NumReceived Length of bytes which have been actually received.
 * \param MaxWait after (seconds) expires.
 * \return Returns 0 in case of errors, 1 instead.
**/
int CComPort::ReadWait(unsigned char *Buffer, int NumToRead, int *NumReceived, double MaxWait)
{
    CCronometer Crono;
    int k;
    double CurrentTime;

    if (!Opened()) return 0;
    if (NumToRead>FBufferMaxLen) return 0;
    Crono.Reset();
    while (1) {
        CurrentTime=Crono.CurrentTime();
        if (!Read(&FBuffer[FBufferLen], FBufferMaxLen-FBufferLen, &k)) return 0;
        FBufferLen+=k;
        if ((FBufferLen>=NumToRead) || (CurrentTime>MaxWait)) { // Completed or TimeOut
            k = (FBufferLen>=NumToRead) ? NumToRead : FBufferLen;
            memcpy((char *)Buffer, (char *) FBuffer, k);
            memcpy((char *)FBuffer, (char *) &FBuffer[k], FBufferLen-k);
            FBufferLen-=k;
            *NumReceived=k;
            return 1;
        }
    }
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
    CCronometer Crono;
    int i, k, stopCounter;
    double CurrentTime;

    if (!Opened()) return 0;
    if (MaxDim>FBufferMaxLen) return 0;
    *FrameAvailable=0;
    i=0;
    stopCounter=0;
    Crono.Reset();
    while (1) {
        CurrentTime=Crono.CurrentTime();  // get current time
        if (!Read(&FBuffer[FBufferLen], FBufferMaxLen-FBufferLen, &k)) return 0;
        FBufferLen+=k;
        while (i<FBufferLen) {
            if ((FBuffer[i]==FEndOfFrame) || (i>=(MaxDim-1))) {
                if (i==0) {
                    FBufferLen--;
                    memcpy((char *)FBuffer, (char *) &FBuffer[1], FBufferLen);
                } else {
                    i++;
                    memcpy((char *)Buffer, (char *) FBuffer, i);
                    Buffer[i+1]=FEndOfFrame; // add another EOF after last char
                    *FrameAvailable = 1;
                    memcpy((char *)FBuffer, (char *) &FBuffer[i], FBufferLen-i);
                    FBufferLen-=i;
                    return 1;
                }
            } else {
                i++;
            }
        }
        if (CurrentTime>MaxWait) {
            stopCounter++;
            if (stopCounter>=10) {
                return 1; // TimeOut
            }
        }
    }
    return 1;
}


/**
 * Check if the port is Open.
 *
 * \return Returns 1 if the port is open, 0 instead.
**/
int CComPort::Opened()
{
    return FOpened;
}


/**
 * Open the Com port.
 *
 * \param PortSpeed Speed of the port in bauds.
 * \return Returns 0 if errors occur, 1 instead.
**/
int CComPort::Open(int PortSpeed)
{
    char Port[20];

#ifdef _WIN32
    FPortSpeed = PortSpeed;
    DCB dcb;
    WCHAR WPort[20];
    COMMTIMEOUTS ComTO;
#ifdef _MSC_VER //Microsoft Visual Studio
    sprintf_s(Port, "//./COM%d", FPortNumber);
#else
	sprintf(Port, "//./COM%d", FPortNumber);
#endif
    ctow(WPort, Port);
#if __GNUC__ >= 4
    hCom = CreateFile( WPort,
#elif defined(_MSC_VER)
	hCom = CreateFile( WPort,
#else
	hCom = CreateFile( Port
#endif
                       GENERIC_READ | GENERIC_WRITE,
                       0,    // comm devices must be opened w/exclusive-access
                       NULL, // no security attributes
                       OPEN_EXISTING, // comm devices must use OPEN_EXISTING
                       0,    // not overlapped I/O
                       NULL  // hTemplate must be NULL for comm devices
                       );
    if (hCom == INVALID_HANDLE_VALUE) return 0;
#endif
#ifdef __linux__
    sprintf(Port,"/dev/ttyACM%d", FPortNumber);
    //sprintf(Port,"/dev/ttyUSB%d", FPortNumber);

    SetPortSpeed(PortSpeed);

    fd = open(Port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        return 0;
    }
#endif
    FOpened = 1;

#ifdef _WIN32
    if (!GetCommState(hCom, &dcb)) {
        Close();
        return 0;
    }
// *********   DEBUG   **************/
//    COMMPROP CommProp;
//    GetCommProperties(hCom, &CommProp);
//    printf("\n%s",CommProp.dwProvCapabilities);
////////////////////////////////
    dcb.BaudRate = FPortSpeed;
    dcb.ByteSize = FPortSize;
    dcb.Parity = FPortParity;
    dcb.StopBits = FPortstopBits;
    //	DWORD err = GetLastError();
    if (!SetCommState(hCom, &dcb))  {
        Close();
        return 0;
    }
    if (!SetCommMask(hCom, EV_RXCHAR)) {
        Close();
        return 0;
    }

    if (!GetCommTimeouts(hCom, &ComTO))  {
        Close();
        return 0;
    }
    ComTO.ReadTotalTimeoutConstant = 10;//1; // 1 ms: returns immediately
    ComTO.ReadIntervalTimeout = 0; //0;
    ComTO.ReadTotalTimeoutMultiplier = 0;
    if (!SetCommTimeouts(hCom, &ComTO)) {
        Close();
        return 0;
    }
#endif
#ifdef __linux__
    tcgetattr(fd, &oldtio);             /* save current serial port settings */

    bzero(&newtio, sizeof(newtio));     /* clear struct for new port settings */

    // set new port settings for canonical input processing
    newtio.c_cflag = FPortSpeed | FPortSize | FPortParity | FPortstopBits | CREAD | CLOCAL;
    cfsetospeed(&newtio, FPortSpeed);
    newtio.c_iflag |= IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;//ICANON;

    newtio.c_cc[VMIN]     = 0;
    newtio.c_cc[VTIME]    = 0;

    tcflush(fd, TCIFLUSH);

    if(tcsetattr(fd, TCSANOW, &newtio)!=0){
        Close(); // set attributes can not be completed
        return 0;
    }
#endif
    FBufferLen=0;
    return 1;
}




