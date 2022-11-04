## __SMARTMIC1 application__

SMARTMIC1 provides a firmware running on STM32 which acquires audio signals 
using four digital MEMS microphones, elaborates them by means of embedded DSP 
libraries and streams the processed audio to both an USB host and a loudspeaker 
connected to the relevant expansion board.
The package includes the following acoustic libraries:
-	AcousticBF library provides an implementation for a real-time adaptive 
	beamforming algorithm: using the audio signals acquired from two digital MEMS 
	microphones, it creates a virtual directional microphone pointing to a fixed 
	direction in space.
-	AcousticEC library provides an implementation for a real-time echo 
	cancellation routine based on the well-known SPEEX implementation of the MDF 
	algorithm. 
-	AcousticSL library <provides an implementation for a real-time sound source 
	localization algorithm: using 2 or 4 signals acquired from digital MEMS 
	microphones, it can estimate the arrival direction of the audio source.

The firmware provides implementation example for STM32 NUCLEO-F446RE board  
equipped with:
-	X-NUCLEO-CCA01M1, an expansion board based on the STA350BW Sound Terminal® 
	2.1-channel high-efficiency digital audio output system.
-	X-NUCLEO-CCA02M2, an evaluation board based on digital MEMS microphones, designed 
	around STMicroelectronics MP34DT06J digital microphones.
-	STEVAL-MIC001Vx, STEVAL-MIC002Vx or STEVAL-MIC003Vx digital microphones.

A sample implementation is available also for BlueCoin starter kit (STEVAL-BCNKT01V1).

A communication infrastructure is provided, in order to control the device status and 
setup the running algorithm from a host PC.


 After the initialization of all the required elements (microphones, audio output, 
 USB communication and streaming) digital MEMS microphone acquisition starts and 
 drives the whole application: when a millisecond of the microphones signal is made 
 available by the BSP layer, several operations are performed concurrently, 
 depending on the current status of the application. Among these we have:
-	Beam Forming
-	Source Localization
-	Acoustic Echo Cancellation
-	dB SPL Estimation

Then the processed audio is streamed to 2 interfaces at the same time: 
-	 USB: the device is recognized as a standard USB stereo microphone by a host 
PC that can be used to record and save the real time audio streaming. The stereo 
track contains the processed signal (L channel) and an omnidirectional microphone 
as a reference (R channel)
-	I2S: an I2S peripheral of the MCU is connected to the STA350BW Sound Terminal® 
device mounted on the X-NUCLEO-CCA01M1 board. In this way the processed signal can 
be reproduced by a loudspeaker connected to the expansion board.

A communication infrastructure, based on the ST-LINK VCP or an ad-hoc composite Audio+VCP 
USB class, is set up in order to establish a link between the  system and the PC. 
This allows to control the demo at runtime (enable/disable the algorithms) and retrieve 
the relevant data computed in the MCU, like the estimated dBSPL level or the direction 
of Arrival of the main audio source as computed by the Open.Audio libraries.

The HCLK is configured at 168 MHz for STM32F446xx Devices.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@note The clock setting is configured to have an high product performance (high clock frequency) 
      so not optimized in term of power consumption.
	  
@note User must uncomment #define BLUECOIN_1720 in audio_application.h to make the example fully working 
	  when using first generation of BlueCoin platform as described in Errata sheet
	  
	  
### __Hardware and Software environment__

  - This example runs on STM32F446xx devices.
    
  - This example has been tested with STMicroelectronics STM32F4xx-Nucleo rev C
    boards and can be easily tailored to any other supported device 
    and development board.

  - For This example to work properly, a 2.2 µF Capacitor must be present on the board in slot C26.
  

### __How to use it ?__

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

