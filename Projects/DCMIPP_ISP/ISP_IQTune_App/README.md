v1.0.0

# ISP IQTune App

This application is running on the target and allow interaction between
the STM32 ISP IQTune software (executed on the host) and the target to
control and tune any ISP parameters.

When the application is launched, you can connect your host computer to
the board USB1 (CN18) connector unsing an USB-C cable.
Then you must execute the STM32 ISP IQTune desktop application software
on you host computer. The main page of the software lists the available
com port. You just need to select the one suitable for your target and you
can start controlling ISP parameters.

# supported camera sensors

The software is natively compatible with the MB1854B IMX335 and STEVAL-66GYMAI
VD66GY camera modules. Simply change the camera module and restart the software.
No software reconfiguration is needed.

Note:<br>
 - For customer camera module, the sensor driver must be added in the Camera
 Middleware and a first tuning must be performed with the STM32 ISP IQTune
 desktop application to generate the final isp_param_conf.h that can be updated.

# ISP siple preview application

To execute the simple preview application, remove the ISP_MW_TUNING_TOOL_SUPPORT
compilation flag from the project configuration and rebuild the software.

# Documentation
[STM32 ISP IQTune: application for sensor image quality tuning](https://wiki.st.com/stm32mcu/wiki/ISP:STM32_ISP_IQTune:_application_for_sensor_image_quality_tuning)
