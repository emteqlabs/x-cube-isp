# Release Notes for X-CUBE-ISP
Copyright &copy; 2024 STMicroelectronics

## Purpose

The X-CUBE-ISP package provides the functionnality to control and tune the
DCMIPP ISP present in the STM32N6xx products.<br>
It contains:<br>
- An embedded ISP Library middleware (running on the target) hosting 2A
  algorithms (Auto Exposure and Auto White Blance) and mechanisms to control
  the ISP and load sensor tuning file.<br>
`X-CUBE-ISP/Middlewares/ST/STM32_ISP_Library`
-  A tuning application (running on the target) that allows to interact
  with the STM32 ISP IQTune desktop application executed on the host computer
  and connected through usb.<br>
`X-CUBE-ISP/Projects/DCMIPP_ISP/ISP_IQTune_App`

To take control of the DCMIPP ISP:<br>
- The host computer must execute the STM32 ISP IQTune desktop application and
connect to the target thanks to an USB-C cable on the USB1 (CN18) connector of
the board.
- The ISP IQTune App must be loaded and launched on the target.

## Key Features

- Control of all ISP parameters via desktop application
- Image analysis function for camera sensor tuning
- Generation of sensor ISP tuning file
- Embedded Auto Exposure algorithm (AE)
- Embedded Auto White Balance algorithm (AWB)
- STM32 ISP IQTune application

## Update history

### Enhancements, new features
Initial release

### Limitations
- In rare situations, Auto White Balance (AWB) algorithm can flicker between 2 AWB profiles\*
<br>
\* Fix is under preparation

## Software components
| Name                         | Version
|-----                         | -------
| CMSIS                        | v5.9.0
| STM32N6xx CMSIS Device       | v1.0.0
| STM32N6xx HAL/LL Drivers     | v1.0.0
| STM32N6570-DK BSP Drivers    | v1.0.0
| BSP Component aps256xx       | v1.0.6
| BSP Component Common         | v7.3.0
| BSP Component mx66uw1g45g    | v1.1.0
| BSP Component rk050hr18      | v1.0.0
| STM32 Camera Middleware      | v1.4.1
| STM32 USBPD Device Library   | v2.11.3
| STM32 ISP Library            | v1.0.1
| ISP IQTune application       | v1.0.0
