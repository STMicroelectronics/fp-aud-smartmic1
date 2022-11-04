/**
******************************************************************************
* @file    sta350bw_reg.h
* @author  SRA - Central Labs
* @version v3.0.0
* @date    6-May-19
* @brief   This file contains definitions for sta350bw_reg.c
*          firmware driver.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STA350BW_REG_H
#define STA350BW_REG_H

#ifdef __cplusplus
extern "C" {
#endif
  
  /* Includes ------------------------------------------------------------------*/
#include "audio.h"

/* Exported types ------------------------------------------------------------*/
extern AUDIO_Drv_t STA350BW_drv;

/** @addtogroup BSP
* @{
*/ 

/** @addtogroup Components
* @{
*/ 

/** @addtogroup STA350BW
* @{
*/

/** @defgroup STA350BW_Exported_Types STA350BW Exported Types
* @{
*/
    
/** @defgroup STA350BW_adsress_define STA350BW address define
* @brief STA350BW address definitions
* @{
*/
#define         STA350BW_ADDRESS_1   ((uint8_t)0x38)
#define         STA350BW_ADDRESS_2    ((uint8_t)0x3A)
/**
* @}
*/


typedef enum 
{
  STA350BW_OK = (uint8_t)0, 
  STA350BW_ERROR = 1 , 
  STA350BW_NOT_IMPLEMENTED = 2
} 
STA350BW_Error_et;

typedef int32_t (*STA350BW_write_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef int32_t (*STA350BW_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  STA350BW_write_ptr  write_reg;
  STA350BW_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} STA350BW_ctx_t;

      
/**
* @}
*/


/** @defgroup STA350BW_Exported_Constants
* @{
*/

/** @defgroup STA350BW_Registers_Mapping
* @brief STA350BW register mapping
* @{
*/

#define STA350BW_MAX_REGISTERS 	              ((uint8_t)0x56)       

/**
* @brief Configuration Register A
* \code
* Read/Write
* Default value: 0x63
* 7 FAULT detect recovery bypass
* 6 TWAB Thermal warning adjustable bypass
* 5 TWRB Thermal warning recovery bypass
* 4,3 IR Interpolatio ratio
* [2:0] MCS Master clock selection
* \endcode
*/ 
#define STA350BW_CONF_REGA                ((uint8_t)0x00)      /*Configuration Register A*/

/**
* @brief Configuration Register B
* \code
* Read/Write
* Default value: 0x80
* 7 C2IM channel 2 input mapping
* 6 C2IM channel 1 input mapping
* 5 DSCKE Delay serial clock enable
* 4 SAIFB Serial data first bit
* [3:0] Serial Input interface format
* \endcode
*/ 
#define STA350BW_CONF_REGB                ((uint8_t)0x01)      /*Configuration Register B*/


/**
* @brief Configuration Register C
* \code
* Read/write
* Default value: 0x09F
* 7 OCRB Overcurrent warning adjustment bypass
* [5:2] CSZx: FFX compensating pulse size
* [1:0] OMx: FFX Output mode
* 0 Clk_Out: Enable HSE on MCO. 0: MCO disable. 1: MCO enable
* \endcode
*/ 
#define STA350BW_CONF_REGC                ((uint8_t)0x02)      /*Configuration Register C*/

/**
* @brief I2C address Configuration Register D
* \code
* Read/write
* Default value: 0x40
* 7 SME soft mute enable  
* 6 ZDE zero detect enable 
* 5 DRC DRC or anti-clipping mode 
* 4 BQL Biquad Link 
* 3 PSL Post scale Link
* 2 DSPB DSP bypass
* 1 DEMP De-Emphasys filter
* 0 HPB High pass filter bypass
* \endcode
*/ 
#define STA350BW_CONF_REGD                ((uint8_t)0x03)      /*Configuration Register D*/

/**
* @brief I2C address Configuration Register E
* \code
* Read/write
* Default value: 0xC2
* 7 SVE soft volume enable 
* 6 ZCE zero crossing enable 
* 5 DCCV variable distorsion compensation
* 4 PWMS PWM speed
* 3 AME AM noise reduction enable
* 2 NSBW Noise shaper bandwidth
* 1 MPC Max Power correction
* 0 MPCV Variable ax power correction
* \endcode
*/ 
#define STA350BW_CONF_REGE                ((uint8_t)0x04)      /*Configuration Register E*/

/**
* @brief I2C address Configuration Register F
* \code
* Read/write
* Default value: 0x5C
* 7 EAPD External Amplifier Power Down
* 6 PWDN device power down 
* 5 ECLE Auto EAPD on clock loss 
* 4 LDTE LRCK double trigger protection
* 3 BCLE Binary out mode clock loss detection
* 2 IDE Invalid Input Detect
* 1,0 OCFG Output configuration
* \endcode
*/ 
#define STA350BW_CONF_REGF                ((uint8_t)0x05)      /*Configuration Register F*/

/**
* @brief I2C address MUTE/Line Out configuration
* \code
* Read/write
* Default value: 0x10
* [7:6] LOC line out configuration
* [5:4] RESERVED
* 3 C3M Channel 3 MUTE
* 2 C2M Channel 2 MUTE
* 1 C1M Channel 1 MUTE
* 0 MMUTE Master Mute
* \endcode
*/ 
#define STA350BW_MUTE                 ((uint8_t)0x06)      /* MUTE / Lineout configuration */

/**
* @brief I2C address Master Volume
* \code
* Read/write
* Default value: 0xFF
* [7:0] Master volume (default -127.5dB)
* \endcode
*/
#define STA350BW_MVOL                 ((uint8_t)0x07)      /* Master Volume */

/**
* @brief I2C address Channel 1 Volume
* \code
* Read/write
* Default value: 0x60
* [7:0] Master volume (default 0.0dB)
* \endcode
*/
#define STA350BW_C1VOL                ((uint8_t)0x08)      /* Channel 1 volume */

/**
* @brief I2C address Channel 2 Volume
* \code
* Read/write
* Default value: 0x60
* [7:0] Master volume (default 0.0dB)
* \endcode
*/
#define STA350BW_C2VOL                ((uint8_t)0x09)      /* Channel 2 volume */

/**
* @brief I2C address Channel 3 Volume
* \code
* Read/write
* Default value: 0x60
* [7:0] Master volume (default 0.0dB)
* \endcode
*/
#define STA350BW_C3VOL	              ((uint8_t)0x0A)      /* Channel 3 volume */

/**
* @brief I2C address AUTO MODE 1
* \code
* Read/write
* Default value: 0x80
* [7:6] RESERVED
* [5:4] AMGC Audio Preset Gain compression
* [3:0] RESERVED
* \endcode
*/ 
#define STA350BW_AUTO1                ((uint8_t)0x0B)      /* Audio Preset 1 register */

/**
* @brief I2C address AUTO MODE 2
* \code
* Read/write
* Default value: 0x00
* [7:4] XO preset crossover filter
* [3:1] AMAMx AM atomode settings
* 0 AMAME AM automode enable
* \endcode
*/ 
#define STA350BW_AUTO2                ((uint8_t)0x0C)      /* Audio Preset 2 register */

/**
* @brief I2C address Channel 1 configuration register
* \code
* Read/write
* Default value: 0x00
* 7,6 C1OM Channel 1 output mapping
* 5,4 C1LS Channel 1  limiter mapping
* 3 C1BO Channel 1 Binary output
* 2 C1VPB Channel 1 volume bypass
* 1 C1EQBP Channel 1 Equalization Bypass
* 0 C1TCB Channel 1 Tone/Control Bypass
* \endcode
*/ 
#define STA350BW_C1CFG                ((uint8_t)0x0E)      /* Channel 1 configuration register */

/**
* @brief I2C address Channel 2 configuration register
* \code
* Read/write
* Default value: 0x40
* 7,6 C2OM Channel 2 output mapping
* 5,4 C2LS Channel 2  limiter mapping
* 3 C2BO Channel 2 Binary output
* 2 C2VPB Channel 2 volume bypass
* 1 C2EQBP Channel 2 Equalization Bypass
* 0 C2TCB Channel 2 Tone/Control Bypass
* \endcode
*/ 
#define STA350BW_C2CFG                ((uint8_t)0x0F)      /* Channel 2 configuration register */

/**
* @brief I2C address Channel 3 configuration register
* \code
* Read/write
* Default value: 0x80
* 7,6 C2OM Channel 3 output mapping
* 5,4 C2LS Channel 3  limiter mapping
* 3 C2BO Channel 3 Binary output
* 2 C2VPB Channel 3 volume bypass
* 1,0 RESERVED
* \endcode
*/ 
#define STA350BW_C3CFG	              ((uint8_t)0x10)      /* Channel 3 configuration register */  

/**
* @brief I2C address Tone control register
* \code
* Read/write
* Default value: 0x77
* [7:4] Treble
* [3:0] Bass
* \endcode
*/ 
#define STA350BW_TONE                ((uint8_t)0x11)      /* Tone control register */

/**
* @brief I2C address Limiter 1 Attack/Release rate register
* \code
* Read/write
* Default value: 0x6A
* [7:4] Limiter 1 Attack rate
* [3:0] Limiter 1 release rate
* \endcode
*/ 
#define STA350BW_L1AR                ((uint8_t)0x12)      /* Limiter 1 Attack/Release rate register */

/**
* @brief I2C address Limiter 1 Attack/Release threshold register
* \code
* Read/write
* Default value: 0x69
* [7:4] Limiter 1 Attack threshold
* [3:0] Limiter 1  release threshold
* \endcode
*/ 
#define STA350BW_L1ATR	              ((uint8_t)0x13)      /* Limiter 1 Attack/Release threshold register */  

/**
* @brief I2C address Limiter 2 Attack/Release rate register
* \code
* Read/write
* Default value: 0x6A
* [7:4] Limiter 2 Attack rate
* [3:0] Limiter 2 Release rate
* \endcode
*/ 
#define STA350BW_L2AR                ((uint8_t)0x14)      /* Limiter 2 Attack/Release rate register */

/**
* @brief I2C address Limiter 2 Attack/Release threshold register
* \code
* Read/write
* Default value: 0x69
* [7:4] Limiter 2 Attack threshold
* [3:0] Limiter 2  release threshold
* \endcode
*/ 
#define STA350BW_L2ATR	              ((uint8_t)0x15)      /* Limiter 2 Attack/Release threshold register */  

/* RAM download*/

/**
* @brief I2C address  Coefficient address register
* \code
* Read/write
* Default value: 0x00
* [7:6] RESERVED
* [5:0] RAM address
* \endcode
*/ 
#define STA350BW_CFADDR               ((uint8_t)0x16)      /* Coefficient address register  */

/**
* @brief I2C address Coefficient b1 data register bits 23:16
* \code
* Read/write
* Default value: 0x00
* [7:0] coefficient b1 bits 23:16
* \endcode
*/ 
#define STA350BW_B1CF1                ((uint8_t)0x17)      /* Coefficient b1 data register bits 23:16 */

/**
* @brief I2C address Coefficient b1 data register bits 15:8
* \code
* Read/write
* Default value: 0x00
* [7:0] coefficient b1 bits 15:8
* \endcode
*/ 
#define STA350BW_B1CF2	              ((uint8_t)0x18)      /* Coefficient b1 data register bits 15:8 */  

/**
* @brief I2C address Coefficient b1 data register bits 7:0
* \code
* Read/write
* Default value: 0x00
* [7:0] coefficient b1 bits 7:0
* \endcode
*/ 
#define STA350BW_B1CF3	              ((uint8_t)0x19)      /* Coefficient b1 data register bits 7:0 */  

/**
* @brief I2C address Coefficient b2 data register bits 23:16
* \code
* Read/write
* Default value: 0x00
* [7:0] coefficient b2 bits 23:16
* \endcode
*/ 
#define STA350BW_B2CF1                ((uint8_t)0x1A)      /* Coefficient b2 data register bits 23:16 */

/**
* @brief I2C address Coefficient b2 data register bits 15:8
* \code
* Read/write
* Default value: 0x00
* [7:0] coefficient b2 bits 15:8
* \endcode
*/ 
#define STA350BW_B2CF2	              ((uint8_t)0x1B)      /* Coefficient b2 data register bits 15:8 */  

/**
* @brief I2C address Coefficient b2 data register bits 7:0
* \code
* Read/write
* Default value: 0x00
* [7:0] Coefficient b2 data bits 7:0
* \endcode
*/ 
#define STA350BW_B2CF3	              ((uint8_t)0x1C)      /* Coefficient b2 data register bits 7:0 */  

/**
* @brief I2C address Coefficient a1 data register bits 23:16
* \code
* Read/write
* Default value: 0x00
* [7:0] Coefficient a1 data bits 23:16
* \endcode
*/ 
#define STA350BW_A1CF1                ((uint8_t)0x1D)      /* Coefficient a1 data register bits 23:16 */

/**
* @brief I2C address Coefficient a1 data register bits 15:8
* \code
* Read/write
* Default value: 0x00
* [7:0] Coefficient a1 data bits 15:8
* \endcode
*/ 
#define STA350BW_A1CF2	              ((uint8_t)0x1E)      /* Coefficient a1 data register bits 15:8 */  

/**
* @brief I2C address Coefficient a1 data register bits 7:0
* \code
* Read/write
* Default value: 0x00
* [7:0] Coefficient a1 data bits 7:0
* \endcode
*/ 
#define STA350BW_A1CF3	              ((uint8_t)0x1F)      /* Coefficient a1 data register bits 7:0 */  

/**
* @brief I2C address Coefficient a2 data register bits 23:16
* \code
* Read/write
* Default value: 0x00
* [7:0] Coefficient a2 data bits 23:16
* \endcode
*/ 
#define STA350BW_A2CF1                ((uint8_t)0x20)      /* Coefficient a2 data register bits 23:16 */

/**
* @brief I2C address Coefficient a2 data register bits 15:8
* \code
* Read/write
* Default value: 0x00
* [7:0] Coefficient a2 data bits 15:8
* \endcode
*/ 
#define STA350BW_A2CF2	              ((uint8_t)0x21)      /* Coefficient a2 data register bits 15:8 */  

/**
* @brief I2C address Coefficient a2 data register bits 7:0
* \code
* Read/write
* Default value: 0x00
* [7:0] Coefficient a2 data bits 7:0
* \endcode
*/ 
#define STA350BW_A2CF3	              ((uint8_t)0x22)      /* Coefficient a2 data register bits 7:0 */  

/**
* @brief I2C address Coefficient b0 data register bits 23:16
* \code
* Read/write
* Default value: 0x00
* [7:0] coefficient b0 bits 23:16
* \endcode
*/ 
#define STA350BW_B0CF1                ((uint8_t)0x23)      /* Coefficient b0 data register bits 23:16 */

/**
* @brief I2C address Coefficient b0 data register bits 15:8
* \code
* Read/write
* Default value: 0x00
* [7:0] Coefficient b0 data bits 15:8
* \endcode
*/ 
#define STA350BW_B0CF2	              ((uint8_t)0x24)      /* Coefficient b0 data register bits 15:8 */  

/**
* @brief I2C address Coefficient b0 data register bits 7:0
* \code
* Read/write
* Default value: 0x00
* [7:0] Coefficient b0 data bits 7:0
* \endcode
*/ 
#define STA350BW_B0CF3	              ((uint8_t)0x25)      /* Coefficient b0 data register bits 7:0 */  

/**
* @brief I2C address Coefficient write/read control register
* \code
* Read/write
* Default value: 0x00
* [7:4] RESERVED
* 3 RA read a complete set of coefficient
* 2 R1 read only one coefficient
* 1 WA write a complete set of coefficient
* 0 W1 write only one coefficient
* \endcode
*/ 
#define STA350BW_CFUD                 ((uint8_t)0x26)      /* Coefficient write/read control register */


/**
* @brief I2C address Variable max power correction 15:8
* \code
* Read/write
* Default value: 0x1A
* [7:0] Coefficient for Variable max power correction 15:8
* \endcode
*/ 
#define STA350BW_MPCC1	              ((uint8_t)0x27)      /* Variable max power correction 15:8 register*/  

/**
* @brief I2C address Variable max power correction 7:0
* \code
* Read/write
* Default value: 0x30
* [7:0] Coefficient for Variable max power correction 7:0
* \endcode
*/ 
#define STA350BW_MPCC2	              ((uint8_t)0x28)      /* Variable max power correction 7:0 register*/  

/**
* @brief I2C address Variable distortion compensation 15:8
* \code
* Read/write
* Default value: 0xF3
* [7:0] Coefficient for Variable distortion compensation 15:8
* \endcode
*/ 
#define STA350BW_DCC1	              ((uint8_t)0x29)      /* Variable distortion compensation 15:8 */  

/**
* @brief I2C address Variable distortion compensation 7:0
* \code
* Read/write
* Default value: 0x33
* [7:0] Coefficient for Variable distortion compensation 7:0
* \endcode
*/ 
#define STA350BW_DCC2	              ((uint8_t)0x2A)      /* Variable distortion compensation 7:0 */  

/**
* @brief I2C address Fault detect recovery constant register 15:8
* \code
* Read/write
* Default value: 0x00
* [7:0] Fault detect recovery constant 15:8
* \endcode
*/ 
#define STA350BW_FDRC1	              ((uint8_t)0x2B)      /* Fault detect recovery constant register 15:8 */  

/**
* @brief I2C address Fault detect recovery constant register 7:0
* \code
* Read/write
* Default value: 0xC0
* [7:0] Fault detect recovery constant 7:0
* \endcode
*/ 
#define STA350BW_FDRC2	              ((uint8_t)0x2C)      /* Fault detect recovery constant register 7:0 */  

/**
* @brief I2C address Status Register
* \code
* Read
* Default value: 0x7F
* 7 PLLUL PLL unlock
* 6 FAULT Fault detected on bridge
* 5 UVFAULT undervoltage fault
* 4 OVFAULT overvoltage fault
* 3 OCFAULT overcurrent fault
* 2 OCWARN overcurrent warning
* 1 TFAULT Thermal fault
* 0 TWARN thermal warning
* \endcode
*/ 
#define STA350BW_STATUS	              ((uint8_t)0x2D)      /* Status Register */  

/**
* @brief I2C address EQ coefficients and DRC configuration register
* \code
* Read/write
* Default value: 0x00
* 7 XOB Crossover filter bypass
* [6:5] RESERVED
* [4:3] AMGC Anti-clipping and DRC preset
* 2 RESERVED
* [1:0] EQ RAM bank selector
* \endcode
*/ 
#define STA350BW_EQCFG	              ((uint8_t)0x31)      /* EQ coefficients and DRC configuration register */  

/**
* @brief I2C address Limiter 1 extended attack threshold register
* \code
* Read/write
* Default value: 0x30
* 7 EATHEN1 Limiter 1 Extended Attack threshold enable
* [6:0] EATH1  Limiter 1 Extended Attack threshold 
* \endcode
*/ 
#define STA350BW_EATH1	              ((uint8_t)0x32)      /* Limiter 1 extended attack threshold register */  

/**
* @brief I2C address Limiter 1 extended release threshold register
* \code
* Read/write
* Default value: 0x30
* 7 ERTHEN1 Limiter 1 Extended Release threshold enable
* [6:0] ERTH1  Limiter 1 Extended Release threshold 
* \endcode
*/ 
#define STA350BW_ERTH1	              ((uint8_t)0x33)      /* Limiter 1 extended release threshold register  */  

/**
* @brief I2C address Limiter 2 extended attack threshold register
* \code
* Read/write
* Default value: 0x30
* 7 EATHEN2 Limiter 2 Extended Attack threshold enable
* [6:0] EATH2  Limiter 2 Extended Attack threshold 
* \endcode
*/ 
#define STA350BW_EATH2	              ((uint8_t)0x34)      /* Limiter 2 extended attack threshold register */  

/**
* @brief I2C address Limiter 2 extended release threshold register
* \code
* Read/write
* Default value: 0x30
* 7 ERTHEN2 Limiter 2 Extended Release threshold enable
* [6:0] ERTH2  Limiter 2 Extended Release threshold 
* \endcode
*/ 
#define STA350BW_ERTH2	              ((uint8_t)0x35)      /* Limiter 2 extended release threshold register */  

/**
* @brief I2C address Extended configuration register
* \code
* Read/write
* Default value: 0x00
* [7:6] MDRC MDRC or EQ DRC selector
* 5 PS48DB Extended post-scale range
* 4 Extended attack rate Limiter 1
* 3 Extended attack rate Limiter 2
* 2 Biquad 5 enable
* 1 Biquad 6 enable 
* 0 Biquad 7 enable
* \endcode
*/ 
#define STA350BW_CONFX	              ((uint8_t)0x36)      /* Extended configuration register */  

/**
* @brief I2C address soft-volume up configuration register
* \code
* Read/write
* Default value: 0x00
* [7:6] RESERVED
* 5 SVUPE Soft volume up enable 
* [4:0] SVUP Soft volume up coefficient
* \endcode
*/ 
#define STA350BW_SVCA	              ((uint8_t)0x37)      /* soft-volume up configuration register */  

/**
* @brief I2C address soft-volume down configuration register
* \code
* Read/write
* Default value: 0x00
* [7:6] RESERVED
* 5 SVDWE Soft volume down enable 
* [4:0] SVDW Soft volume down coefficient
* \endcode
*/ 
#define STA350BW_SVCB	              ((uint8_t)0x38)      /* soft-volume down configuration register */  


/**
* @brief I2C address DRC RMS filter coefficient c0 23:16 register
* \code
* Read/write
* Default value: 0x01
* [7:0] R_C0 DRC RMS filter coefficient c0 23:16
* \endcode
*/ 
#define STA350BW_RMS0A	              ((uint8_t)0x39)      /* DRC RMS filter coefficient c0 23:16 register */  

/**
* @brief I2C address DRC RMS filter coefficient c0 15:8 register
* \code
* Read/write
* Default value: 0xEE
* [7:0] R_C0 DRC RMS filter coefficient c0 15:8
* \endcode
*/ 
#define STA350BW_RMS0B	              ((uint8_t)0x3A)      /* DRC RMS filter coefficient c0 15:8 register */  

/**
* @brief I2C address DRC RMS filter coefficient c0 7:0 register
* \code
* Read/write
* Default value: 0xFF
* [7:0] R_C0 DRC RMS filter coefficient c0 7:0
* \endcode
*/ 
#define STA350BW_RMS0C	              ((uint8_t)0x3B)      /* DRC RMS filter coefficient c0 7:0 register */  

/**
* @brief I2C address DRC RMS filter coefficient c1 23:16 register
* \code
* Read/write
* Default value: 0x7E
* [7:0] R_C1 DRC RMS filter coefficient c0 23:16
* \endcode
*/ 
#define STA350BW_RMS1A	              ((uint8_t)0x3C)      /* DRC RMS filter coefficient c1 23:16 register */  

/**
* @brief I2C address DRC RMS filter coefficient c1 15:8 register
* \code
* Read/write
* Default value: 0xC0
* [7:0] R_C1 DRC RMS filter coefficient c1 15:8
* \endcode
*/ 
#define STA350BW_RMS1B	              ((uint8_t)0x3D)      /* DRC RMS filter coefficient c1 15:8 register */  

/**
* @brief I2C address DRC RMS filter coefficient c1 7:0 register
* \code
* Read/write
* Default value: 0x26
* [7:0] R_C0 DRC RMS filter coefficient c1 7:0
* \endcode
*/ 
#define STA350BW_RMS1C	              ((uint8_t)0x3E)      /* DRC RMS filter coefficient c1 7:0 register */  

/**
* @brief I2C address Extra volume resolution configuration register
* \code
* Read/write
* Default value: 0x00
* 7 VRESEN Extra volume resolution enable
* 6 VRESTG Extra volume resolution update
* [5:4] C3VR Channel 3 extra volume value
* [3:2] C2VR Channel 2 extra volume value
* [1:0] C1VR Channel 1 extra volume value
* \endcode
*/ 
#define STA350BW_EVOLRES              ((uint8_t)0x3F)      /* Extra volume resolution configuration register */  

/**
* @brief I2C address Quantization error noise correction register
* \code
* Read/write
* Default value: 0x00
* 7 Quntization Noise shaping enable
* 6 Quntization Noise shaping on biquad 7 
* 5 Quntization Noise shaping on biquad 6 
* 4 Quntization Noise shaping on biquad 5 
* 3 Quntization Noise shaping on biquad 4 
* 2 Quntization Noise shaping on biquad 3 
* 1 Quntization Noise shaping on biquad 2 
* 0 Quntization Noise shaping on biquad 1
* \endcode
*/ 
#define STA350BW_NSHAPE	              ((uint8_t)0x48)      /* Quantization error noise correction register */  

/**
* @brief I2C address Extended coefficient range up to -4...4 biquad 1-4 register
* \code
* Read/write
* Default value: 0x00
* [7:6] CXTB4 Extended coefficient on biquad 4
* [5:4] CXTB3 Extended coefficient on biquad 3
* [3:2] CXTB2 Extended coefficient on biquad 2
* [1:0] CXTB1 Extended coefficient on biquad 1
* \endcode
*/ 
#define STA350BW_CXT_B4B1             ((uint8_t)0x49)      /* Extended coefficient range up to -4...4 biquad 1-4 register */  

/**
* @brief I2C address Extended coefficient range up to -4...4 biquad 5-7 register
* \code
* Read/write
* Default value: 0x00
* [7:6] RESERVED
* [5:4] CXTB7 Extended coefficient on biquad 7
* [3:2] CXTB6 Extended coefficient on biquad 6
* [1:0] CXTB5 Extended coefficient on biquad 5
* \endcode
*/ 
#define STA350BW_CXT_B7B5             ((uint8_t)0x4A)      /* Extended coefficient range up to -4...4 biquad 5-7 register */  

/**
* @brief I2C address Miscellaneous register 1
* \code
* Read/write
* Default value: 0x04
* 7 RPDNEN Rate powerdown enable
* 6 NSHHPEN Noise shaping feature enable 
* 5 BRIDGOFF Bridge immediate OFF  
* [4:3] RESERVED 
* 2 CPWMEN Channel PWM enable
* [1:0] RESERVED 
* \endcode
*/ 
#define STA350BW_MISC1 	              ((uint8_t)0x4B)      /* Miscellaneous register 1 */  

/**
* @brief I2C address Miscellaneous register 2
* \code
* Read/write
* Default value: 0x00 
* [7:5] RESERVED 
* [4:2] PNDLSL Power-down delay selector
* [1:0] RESERVED 
* \endcode
*/ 
#define STA350BW_MISC2 	              ((uint8_t)0x4C)      /* Miscellaneous register 2 */  

/**
* @}
*/  



/** @defgroup STA350BW_Main_parameter 
* @{
*/
#define STA350BW_EAPD_ON	((uint8_t)0x80)
#define STA350BW_EAPD_OFF	((uint8_t)0x00)
#define STA350BW_PWDN_OFF	((uint8_t)0x40) 
#define STA350BW_PWDN_ON	((uint8_t)0x00) /* low power consumption */ 

#define STA350BW_MVOL_0dB	((uint8_t)0x00) 
#define STA350BW_MVOL_MUTE	((uint8_t)0xFF) 
/**
* @}
*/  


/** @defgroup STA350BW_Input_frequency_selection 
* @{
*/
#define         STA350BW_Fs_32000                       ((uint32_t)32000)
#define         STA350BW_Fs_44100                       ((uint32_t)44100)
#define         STA350BW_Fs_48000                       ((uint32_t)48000)
#define         STA350BW_Fs_88200                       ((uint32_t)88200)
#define         STA350BW_Fs_96000                       ((uint32_t)96000)    

#define         STA350BW_MCLK_256_LR_48K                ((uint8_t)0x03)
#define         STA350BW_MCLK_128_LR_48K                ((uint8_t)0x04)
#define         STA350BW_MCLK_256_LR_96K                ((uint8_t)0x09)
#define         STA350BW_MCLK_128_LR_96K                ((uint8_t)0x0B)
/**
* @}
*/  





/** @defgroup STA350BW_mode_selection
* @brief STA350BW mode configuration constants
* @{
*/
#define         STA350BW_STEREO_CONF                   ((uint8_t)0x00)
#define         STA350BW_2SE_1BTL_CONF                 ((uint8_t)0x01)
#define         STA350BW_STEREO_EXT_BRIDGE_CONF        ((uint8_t)0x00)
#define         STA350BW_MONOBTL_CONF                  ((uint8_t)0x11)
#define         STA350BW_BINARY_CONF                   ((uint8_t)0x80)	/* on registers 0E, 0F, 10 */ 
/**
* @}
*/

/** @defgroup STA350BW_DSP_option_selection
* @brief STA350BW constants related to data path management
* @{
*/
#define         STA350BW_DSPB                          ((uint8_t)0x00)
#define         STA350BW_C1EQBP                        ((uint8_t)0x01)
#define         STA350BW_C2EQBP                        ((uint8_t)0x02)
#define         STA350BW_C1TCB                         ((uint8_t)0x03)
#define         STA350BW_C2TCB                         ((uint8_t)0x04)
#define         STA350BW_C1VBP                         ((uint8_t)0x05)
#define         STA350BW_C2VBP                         ((uint8_t)0x06)   
#define         STA350BW_HPB                           ((uint8_t)0x07)
#define         STA350BW_DEMP                          ((uint8_t)0x08)
#define         STA350BW_BQL                           ((uint8_t)0x09)
#define         STA350BW_BQ5                           ((uint8_t)0x0A)
#define         STA350BW_BQ6                           ((uint8_t)0x0B)
#define         STA350BW_BQ7                           ((uint8_t)0x0C)
#define         STA350BW_EXT_RANGE_BQ1                 ((uint8_t)0x0D)
#define         STA350BW_EXT_RANGE_BQ2                 ((uint8_t)0x0E)
#define         STA350BW_EXT_RANGE_BQ3                 ((uint8_t)0x0F)
#define         STA350BW_EXT_RANGE_BQ4                 ((uint8_t)0x10)
#define         STA350BW_EXT_RANGE_BQ5                 ((uint8_t)0x11)
#define         STA350BW_EXT_RANGE_BQ6                 ((uint8_t)0x12)
#define         STA350BW_EXT_RANGE_BQ7                 ((uint8_t)0x13)
#define         STA350BW_RAM_BANK_SELECT               ((uint8_t)0x14)
/**
* @}
*/


/** @defgroup STA350BW_state_define STA350BW state define
* @brief STA350BW state definitions
* @{
*/   
#define         STA350BW_ENABLE                          ((uint8_t)0x01)
#define         STA350BW_DISABLE                         ((uint8_t)0x00)
#define         STA350BW_RANGE_ONE                       ((uint8_t)0x01)
#define         STA350BW_RANGE_TWO                       ((uint8_t)0x02)
#define         STA350BW_RANGE_FOUR                      ((uint8_t)0x04)
/**
* @}
*/

/** @defgroup STA350BW_channel_define STA350BW channel define
* @brief STA350BW channels definitions
* @{
*/  
#define       STA350BW_CHANNEL_MASTER                             ((uint8_t)0x00)
#define       STA350BW_CHANNEL_1                            ((uint8_t)0x01)
#define       STA350BW_CHANNEL_2                             ((uint8_t)0x02)
#define       STA350BW_CHANNEL_3                             ((uint8_t)0x03)
/**
* @}
*/

/** @defgroup STA350BW_channel_define STA350BW Biq define
* @brief STA350BW Biq definitions
* @{
*/  
#define       STA350BW_RAM_BANK_FIRST                             ((uint8_t)0x00)
#define       STA350BW_RAM_BANK_SECOND                            ((uint8_t)0x01)
#define       STA350BW_RAM_BANK_THIRD                             ((uint8_t)0x02)
#define       STA350BW_CH1_BQ1                                    ((uint8_t)0x00)
#define       STA350BW_CH1_BQ2                                    ((uint8_t)0x01)
#define       STA350BW_CH1_BQ3                                    ((uint8_t)0x02)
#define       STA350BW_CH1_BQ4                                    ((uint8_t)0x03)
#define       STA350BW_CH2_BQ1                                    ((uint8_t)0x04)
#define       STA350BW_CH2_BQ2                                    ((uint8_t)0x05)
#define       STA350BW_CH2_BQ3                                    ((uint8_t)0x06)
#define       STA350BW_CH2_BQ4                                    ((uint8_t)0x07)

/**
* @}
*/
  

/** @defgroup STA350BW_Exported_Functions
* @{
*/


int32_t sta350bw_Init(STA350BW_ctx_t * handle, uint16_t volume, uint32_t samplingFreq); 
int32_t sta350bw_DeInit(STA350BW_ctx_t * handle);
int32_t sta350bw_ReadID(STA350BW_ctx_t * handle);
int32_t sta350bw_Play(STA350BW_ctx_t * handle, uint16_t* pBuffer, uint16_t Size);
int32_t sta350bw_Pause(STA350BW_ctx_t * handle);
int32_t sta350bw_Resume(STA350BW_ctx_t * handle);
int32_t sta350bw_SetMute(STA350BW_ctx_t * handle, uint8_t channel, uint8_t state);
int32_t sta350bw_SetVolume(STA350BW_ctx_t * handle, uint32_t channel, uint8_t Volume) ;
int32_t sta350bw_SetFrequency(STA350BW_ctx_t * handle, uint32_t AudioFreq) ;
int32_t sta350bw_Stop(STA350BW_ctx_t * handle);
int32_t sta350bw_Reset(STA350BW_ctx_t * handle);
int32_t sta350bw_SetEq(STA350BW_ctx_t * handle, uint8_t ramBlock, uint8_t filterNumber, uint32_t * filterValues);
int32_t sta350bw_SetTone(STA350BW_ctx_t * handle, uint8_t toneGain);
int32_t sta350bw_SetDSPOption(STA350BW_ctx_t * handle, uint8_t option, uint8_t state);

  
/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#ifdef __cplusplus
}
#endif

#endif /*STA350BW_REG_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
