# FP-AUD-SMARTMIC1 Firmware Package

![latest tag](https://img.shields.io/github/v/tag/STMicroelectronics/fp-aud-smartmic1.svg?color=brightgreen)

**FP-AUD-SMARTMIC1** provides a firmware running on STM32 which acquires audio signals of four digital MEMS microphones, elaborates them by means of embedded
DSP libraries and streams the processed audio to both an USB host and a loudspeaker connected to the relevant expansion board. A communication infrastructure 
is provided as well, allowing the control of the device status and the setup of the running algorithm from a host PC.

FP-AUD-SMARTMIC1 software features:

- Complete middleware to build audio applications using ST digital MEMS microphones and **STA350BW** Sound Terminal® 2.1-channel high-efficiency digital audio output system.
- **AcousticBF** library provides an implementation for a real-time adaptive beamforming algorithm 
- **AcousticEC** library provides an implementation for a real-time echo cancellation 
- **AcousticSL** library provides an implementation for a real-time sound source localization algorithm 
- dB SPL estimation
- Communication to a host PC via STDCmdP protocol and dedicated PC software
- Easy portability across different MCU families thanks to STM32Cube
- Free user-friendly license terms

Examples implementation available for:

-  **NUCLEO-F446RE** board equipped with **X-NUCLEO-CCA01M1**, an expansion board based on the STA350BW Sound Terminal® 2.1-channel high-efficiency digital audio 
output system, **X-NUCLEO-CCA02M2**, an evaluation board based on ST digital MEMS microphones and STEVAL-MIC001Vx, STEVAL-MIC002Vx or STEVAL-MIC003Vx digital microphones.
-  **STEVAL-BCNKT01V1 BlueCoin** kit

The figure below shows the overall architecture.

![](_htmresc/FP-AUD-SMARTMIC1_Software_Architecture.png)

Here is the list of references to user documents:

- [UM2219](https://www.st.com/resource/en/user_manual/dm00393676.pdf) : Getting started with STM32 ODE function pack for MEMS microphones acquisition, advanced audio processing and audio output
- [UM2212](https://www.st.com/resource/en/user_manual/dm00390468.pdf) : Getting started with Acoustic SL real-time sound source localization middleware
- [UM2213](https://www.st.com/resource/en/user_manual/dm00390471.pdf) : Getting started with AcousticEC real-time acoustic echo cancellation middleware
- [UM2214](https://www.st.com/resource/en/user_manual/dm00391112.pdf) : Getting started with AcousticBF real-time beam forming middleware
- [STM32Cube](https://www.st.com/stm32cube) : STM32Cube
- [STM32 Nucleo boards](https://www.st.com/stm32nucleo) : STM32 Nucleo boards
- [STM32 Nucleo expansion boards](https://www.st.com/x-nucleo) : STM32 Nucleo expansion boards

## Known Limitations

- None

## Development Toolchains and Compilers

-   IAR Embedded Workbench for ARM (EWARM) toolchain V9.20.1
-   RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.37
-   STM32CubeIDE Version 1.10.1

## Supported Devices and Boards

- STM32F446RE devices
- [STEVAL-BCNKT01V1](https://www.st.com/content/st_com/en/products/evaluation-tools/solution-evaluation-tools/sensor-solution-eval-boards/steval-bcnkt01v1.html)
- [NUCLEO-F446RE](https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-nucleo-boards/nucleo-f446re.html) Rev C
- [X-NUCLEO-CCA01M1](https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32-nucleo-expansion-boards/stm32-ode-translate-hw/x-nucleo-cca01m1.html)
- [X-NUCLEO-CCA02M2](https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32-nucleo-expansion-boards/stm32-ode-sense-hw/x-nucleo-cca02m2.html)

## Backward Compatibility

- User must recompile the application to be fully working when using first generation of BlueCoin platform as described in Errata Sheet [ES0462](https://www.st.com/resource/en/errata_sheet/es0462-software-limitations-related-to-stevalbcnkt01v1-lot-1720-stmicroelectronics.pdf)

## Dependencies

- None
