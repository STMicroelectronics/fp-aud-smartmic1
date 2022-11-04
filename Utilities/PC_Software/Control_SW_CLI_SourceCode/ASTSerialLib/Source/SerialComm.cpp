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
  
#include "SerialComm.h"

#if !defined (_WIN32) || (_WIN64)
#include "string.h"
#endif

SerialComm::SerialComm(void)
{
	CComPort();
	SetEndOfFrame(TMsg_EOF);
	FDevAddr=0;
}

SerialComm::~SerialComm()
{
}

/**
* Sets the Device Address of the host on the serial bus.
*
* \param Value Device Address value 0-255.
**/
int32_t SerialComm::SetDevAddr(int32_t Value)
{
	if ((Value>=0) && (Value<=255)) {
		FDevAddr=Value;
		return 1;
	}
	return 0;
}

/**
* Sets the end of frame character for AT commands.
**/
int32_t SerialComm::SetEndOfFrame_AT()
{
	SetEndOfFrame('\r');
	return 0;
}

/**
* Sets the end of frame character for AST Serial Protocol.
**/
int32_t SerialComm::SetEndOfFrame_ASTprotocol()
{
	SetEndOfFrame(TMsg_EOF);
	return 0;
}

/**
* Get the Device Address of the host on the serial bus.
*
* \return Returns Value Device Address value 0-255.
**/
int32_t SerialComm::GetDevAddr(void)
{
	return FDevAddr;
}

/**
* Receive a TMsg message from the serial bus.
*
* \param Msg Pointer to TMsg, it will contain the received message.
* \param MaxWait Maximum wait allowed for reception.
* \return Returns 0 if errors occur, 1 instead.
**/
int32_t SerialComm::ReceivedMSG(TMsg *Msg, double MaxWait)
{
	uint8_t My_Buffer[2*TMsg_MaxLen];
	int32_t Available;

	if (!Opened())
		return 0;
	if (!ReadFrame(My_Buffer, 2*TMsg_MaxLen, &Available, MaxWait))
		return 0;
	if (!Available)
		return 0;
	if (!ReverseByteStuffCopy(Msg, My_Buffer))
		return 0;
	if (!CHK_CheckAndRemove(Msg))
		return 0;

	return 1;
}

/**
* Send a Buffer message.
*
* \param My_Buffer Pointer to unsigned char, it contains message to be send.
* \param len = lenght of My_Buffer
* \return Returns 0 if errors occur, 1 instead.
**/
int32_t SerialComm::SendBuffer(unsigned char *My_Buffer, int32_t len)
{
	int32_t i;

    if (!Opened()) return 0;

	if (!Write(My_Buffer, len, &i)) return 0;
	if (len != i) return 0;
	return 1;
}

/**
* Send a string
*
* \param My_Buffer Pointer to a char buffer that contains a string which ends with \\0
* \return Returns 0 if errors occur, 1 instead.
**/
int32_t SerialComm::SendString(char *String)
{
	int32_t i;

	if (!Opened()) return 0;
	int32_t len = (int32_t)strlen(String);
	if (!Write((unsigned char *)String, len, &i)) return 0;
	if (len != i) return 0;
	return 1;
}

int32_t SerialComm::ReceiveString(char *String, int32_t MaxLen, double MaxWait)
{
	int32_t Available;

	if (!Opened())
		return 0;
	if (!ReadFrame((unsigned char *)String, MaxLen, &Available, MaxWait))
		return 0;
	if (!Available)
		return 0;

	// Add string terminator
	int32_t i=0;
	while(String[i]!='\r') i++;
	String[i+1]=0;

	return 1;
}

/**
* Send a TMsg message.
*
* \param Msg Pointer to TMsg, it contains message to be send.
* \return Returns 0 if errors occur, 1 instead.
**/
int32_t SerialComm::SendMsg(TMsg *Msg)
{
	int32_t Count, i;
	uint8_t My_Buffer[2*TMsg_MaxLen];

	if (!Opened()) return 0;
	CHK_ComputeAndAdd(Msg);
	Count=ByteStuffCopy(My_Buffer, Msg);
	if (!Write(My_Buffer, Count, &i)) return 0;
	if (Count != i) return 0;
	return 1;
}

/**
* Send a command to the serial device.
*
* \param Addr Address of the device for serial communication.
* \param Cmd Command to be issued.
* \param DataLen Length of the Data array (in byte).
* \param Data Array of byte, payload of command Cmd.
* \return Returns -1 if errors occur, 1 instead.
**/
int32_t SerialComm::SendSerialCmd(uint8_t Addr, uint8_t Cmd, uint32_t DataLen, uint8_t *Data)
{
	TMsg Msg;
	uint32_t i;

	Msg.Len=3+DataLen;
    Msg.Data[0]=Addr;
	Msg.Data[1]=FDevAddr;
	Msg.Data[2]=Cmd;
	for(i=0;i<DataLen;i++){
		Msg.Data[3+i]=Data[i];
	}
	if (!SendMsg(&Msg)) return -1;
	return 1;
}

/**
* Receive a command from the serial device.
*
* \param Addr of the device for serial communication.
* \param Cmd Command issued.
* \param MSTimeout Maximum waiting time for the reply.
* \param MaxLen Maximum length of the Data array (in byte).
* \param Data Array of byte, payload of command Cmd.
* \return Returns -1 if errors occur, datalength if no errors occurs.
**/
int32_t SerialComm::ReceiveSerialCmdReply(uint8_t Addr, uint8_t Cmd, u16 MSTimeout, uint32_t MaxLen, uint8_t *Data)
{
	TMsg Msg;
	uint32_t i;

	do {
		if (!ReceivedMSG(&Msg, ((double)MSTimeout)/1000.0))
			return -1;
		if (((int32_t)Msg.Len)<3)
			return -1;
		if (Msg.Data[0]!=FDevAddr)
			return -1;
        if (Msg.Data[1]!=Addr)
			return -1;
	} while(Msg.Data[2]!=(Cmd+CMD_Reply_Add)
		&& Msg.Data[2]!=(Cmd));

	if(Data==0 && Msg.Len!=3)
		return -1;
	for(i=0;i<(Msg.Len-3);i++){
		if(i>=(uint32_t)MaxLen)
			return -1;
		Data[i]=Msg.Data[3+i];
	}
	return Msg.Len-3;
}

/**
* Receive a command in asynchronous mode from the serial device.
*
* \param Addr of the device for serial communication.
* \param CmdReceived Command received.
* \param MSTimeout Maximum waiting time for the reply.
* \param MaxLen Maximum length of the Data array (in byte).
* \param Data Array of byte, payload of command Cmd.
* \return Returns -1 if errors occur, datalength if no errors occurs.
**/
int32_t SerialComm::ReceiveSerialCmdAsync(uint8_t Addr, uint8_t *CmdReceived, u16 MSTimeout, uint32_t MaxLen, uint8_t *Data)
{
	TMsg Msg;
	uint32_t i;

	if (!ReceivedMSG(&Msg, ((double)MSTimeout)/1000.0))
		return -1;
	if (((int32_t)Msg.Len)<3)
		return -1;
	if (Msg.Data[0]!=FDevAddr)
		return -1;
    if (Msg.Data[1]!=Addr)
		return -1;
	*CmdReceived = Msg.Data[2];
	if(Data==0 && Msg.Len!=3)
		return -1;
	for(i=0;i<(Msg.Len-3);i++){
		if(i>=(uint32_t)MaxLen)
			return -1;
		Data[i]=Msg.Data[3+i];
	}
	return Msg.Len-3;
}

int32_t SerialComm::DeleteAllReceivedMessages(uint8_t Addr, uint8_t Cmd, u16 MSTimeout, uint32_t MaxLen, uint8_t *Data){
	// Delete all the received message, check the last one if any
	TMsg Msg;
	uint32_t i;
	char atLeastOne;
	atLeastOne = 0;
	while (ReceivedMSG(&Msg, ((double)MSTimeout)/1000.0)){
		atLeastOne = 1;
	}
	if (atLeastOne){
		if (((int32_t)Msg.Len)<3)
			return -1;
		if (Msg.Data[0]!=FDevAddr)
			return -1;
        if (Msg.Data[1]!=Addr)
			return -1;
		if (Msg.Data[2]!=(Cmd+CMD_Reply_Add))
			return -1;
		if(Data==0 && Msg.Len!=3)
			return -1;
		for(i=0;i<(Msg.Len-3);i++){
			if(i>=(uint32_t)MaxLen)
				return -1;
			Data[i]=Msg.Data[3+i];
		}
		return Msg.Len-3;
	} else {
		return 0;
	}

}
