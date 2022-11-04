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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STCmdP_COMMAND_H
#define __STCmdP_COMMAND_H



/**********  GENERIC  CMD  (0x00 - 0x0F)  **********/

//												0x00
#define CMD_PING								0x01
#define CMD_Ping								0x01
#define CMD_Read_PresString						0x02
#define CMD_NACK								0x03
#define CMD_InterfaceType						0x04
//												0x05
//												0x06
#define CMD_Start_ExpData_Streaming             0x07
#define CMD_Start_Data_Streaming                0x08
#define CMD_Stop_Data_Streaming		        	0x09
#define CMD_StartDemo							0x0A
#define CMD_Sleep_Sec							0x0B
#define CMD_Set_DateTime						0x0C
#define CMD_Get_DateTime						0x0D
#define CMD_Enter_DFU_Mode						0x0E
#define CMD_Reset								0x0F

#define CMD_Reply_Add							0x80

/****************************************************/


/*********** CAMERA  CMD  (0x10 - 0x3F)  ************/

#define CMD_CameraGetState						0x10
#define CMD_CameraStart							0x11
#define CMD_CameraStop							0x12
#define CMD_CameraGetImage						0x13
#define CMD_CameraContinue						0x14
#define CMD_CameraGetImageAtOnce				0x15
#define CMD_CameraI2cWrite						0x16
#define CMD_CameraI2cRead						0x17
#define CMD_CameraFrameCount           			0x18
#define CMD_CameraCheckI2CCommunication			0x19
#define CMD_CameraGetExposureTimeus    			0x1A
#define CMD_CameraSwitchCALConf					0x1B
//												0x1C
#define CMD_RVS_Ready							0x1D
//												0x1E
//												0x1F
#define CMD_SDCardReadDirAtOnce					0x20
#define CMD_SDCardReadFileAtOnce				0x21
#define CMD_SDCardReadLastImageAtOnce			0x22
#define CMD_SDCardReadThumbAtOnce				0x23
#define CMD_SDCardReadLastThumbAtOnce			0x24
#define CMD_FDGetFaces							0x25
#define CMD_HarrisGetPoints						0x26
#define CMD_HarrisSettings						0x27
#define CMD_FastGetPoints						0x28
#define CMD_FastSettings						0x29
#define CMD_ColDetGetColor						0x2A
#define CMD_ColColorDetSettings					0x2B
//												0x2C
//												0x2D
//												0x2E
//												0x2F
#define CMD_SelectDemo							0x30
#define CMD_BlobSearchStart						0x31
#define CMD_BlobSearchSetOptions				0x32
#define CMD_BlobSearchSetClassParam				0x33
#define CMD_BlobSearchGetBlobs					0x34
#define CMD_BlobSearchFmiSaveBS					0x35
#define CMD_BlobSearchFmiLoadBS					0x36
#define CMD_BlobSearchRun						0x37
#define CMD_BlobSearchGetVal					0x38
#define CMD_BlobSearchGetBSParam				0x39
#define CMD_RGBCubeClear						0x3A
#define CMD_RGBCubeSetClassR					0x3B
#define CMD_RGBCubeSetClassG					0x3C
#define CMD_RGBCubeSetClassB					0x3D
#define CMD_RGBCubeSetClassCube					0x3E


/****************************************************/


/*********** AUDIO  CMD  (0x3F - 0x4F)  ************/

#define CMD_AudioModule_SetStatus				0x40
#define CMD_AudioModule_GetStatus				0x41
#define CMD_AudioModule_GetSourceLocAngle		0x42
#define CMD_AudioModule_GetEnergydB				0x43
#define CMD_AudioModule_GetConfigMic    	    0x44
#define CMD_AudioModule_Audio_Init			    0x45
#define CMD_AudioModule_SetVolume				0x46
#define CMD_AudioModule_GetVolume 				0x47
#define CMD_AudioModule_GetStatusBeam			0x48
#define CMD_AudioModule_EnableBeam				0x49
#define CMD_AudioModule_DisableBeam				0x4A
#define CMD_AudioModule_EnableBeamToSource		0x4B
#define CMD_AudioModule_DisableBeamToSource		0x4C
#define CMD_AudioModule_GetStatusSourceLoc		0x4D
#define CMD_AudioModule_EnableSourceLoc			0x4E
#define CMD_AudioModule_DisableSourceLoc 		0x4F

/****************************************************/


/********** BABYBEAR/AUDIO  CMD  (0x50 - 0x5F)  ***********/

#define CMD_BBx_Init							0x50
#define CMD_BBx_ReadDistance					0x51
#define CMD_BB_ReadData							0x52
#define CMD_BB_GestureRecognition				0x53
#define CMD_BB_ReInit							0x54
#define CMD_BB_StartGestureRecognition			0x55
#define CMD_BB_StopGestureRecognition   		0x56
#define CMD_AudioModule_CheckStopApp     		0x57
#define CMD_AudioModule_VestecCMD				0x58
#define CMD_AudioModule_StopAlgorithms			0x59
#define CMD_AudioModule_GetStatusBeamToSource	0x5A
#define CMD_AudioModuleCmd_GetEnhancement   	0x5B
#define CMD_AudioModuleCmd_SetEnhancement		0x5C
#define CMD_AudioModule_EnableASR				0x5D
#define CMD_AudioModule_DisableASR				0x5E
#define CMD_AudioModule_GetASRCommand			0x5F

/****************************************************/


/******** ENVIRONMENTAL  CMD  (0x60 - 0x6F)  ********/

#define CMD_LPS25H_Init							0x60
#define CMD_LPS25H_Read							0x61
#define CMD_HTS221_Init							0x62
#define CMD_HTS221_Read							0x63
#define CMD_UVIS25_Init							0x64
#define	CMD_UVIS25_Read							0x65
//			 									0x66
#define CMD_AudioModule_GetSourceLocResolution	0x67
#define CMD_AudioModule_SetSourceLocResolution	0x68
#define CMD_AudioModule_GetSourceLocThreshold												0x69
#define CMD_AudioModule_SetSourceLocThreshold												0x6A
#define CMD_AudioModule_GetSourceLocFilterP												0x6B
#define CMD_AudioModule_SetSourceLocFilterP												0x6C
//												0x6D
#define CMD_AltitudeCalc_Start					0x6E
#define CMD_AltitudeCalc_Stop					0x6F

/****************************************************/


/******** INERTIAL  CMD  (0x70 - 0x/7F)  ********/

#define CMD_LSM9DS1_Init						0x70
#define CMD_LSM9DS1_9AXES_Read					0x71
#define CMD_SF_Stop					        	0x72
#define CMD_LSM9DS1_ACC_Read                    0x73
#define CMD_LSM9DS1_GYR_Read 					0x74
#define CMD_LSM9DS1_MAG_Read 					0x75
#define CMD_LSM6DS0_Init                        0x76
#define CMD_LSM6DS0_ACC_Read                    0x77
#define CMD_LSM6DS0_GYR_Read 					0x78
#define CMD_LIS3MDL_Init						0x79
#define CMD_LIS3MDL_Read						0x7A
//												0x7B
#define CMD_SF_Init                             0x7C
#define CMD_SF_Data                             0x7D
//												0x7E
//												0x7F

/****************************************************/





#endif /* __STCmdP_COMMAND_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
