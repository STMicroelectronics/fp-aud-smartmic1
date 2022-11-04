/**
******************************************************************************
* @file    STCmdP_Command.h
* @author  Central Labs
* @version V1.0.0
* @date    11-Mar-2015
* @brief   This file contains commands code 
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STCmdP_COMMAND_H
#define __STCmdP_COMMAND_H

#define CMD_PING								0x01
#define CMD_Ping								0x01
#define CMD_Read_PresString						        0x02
#define CMD_NACK								0x03
#define CMD_InterfaceType						        0x04
#define CMD_Enter_DFU_Mode						        0x0E
#define CMD_Reset								0x0F
#define CMD_Reply_Add							        0x80

#define CMD_AudioModule_SetStatus				                0x40
#define CMD_AudioModule_GetStatus				                0x41


#endif /* __STCmdP_COMMAND_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
