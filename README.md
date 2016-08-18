This firmware is part of my master thesis. It combines UAVCAN protocol implemented in C++ and STM32 HAL libraries to create a CAN node on a high frequency 122 Ghz radar module which communicates 32 bit floating point unit data to Pixhawk flight controller.

The intended use is to check the performance of the radar in dynamic conditions (flight, rough weather etc) and also to implement a collision avoidance control loop.

The target MCU is STM32F437VI on baremetal. 
The toolchain used is ARM baremetal.
The build system is GNU make
