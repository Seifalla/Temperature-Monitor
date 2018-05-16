# Temperature-Monitor

# Platform:

This software runs on the TI EK-TM4C123GXL Microcontroller.

# Description:

This program reads the temperatures measured by the heat sensor and sends the readings to the computer. The computer displays the readings on the screen. The flash rate of the light-emitting diode (LED) varies as a function of temperature.

# Implementation:

This program configures an analog-to-digital converter to receive the readings from the micro-controller's internal temperature sensor. The raw data is converted to Celsius temperature and transferred over a serial connection (UART).
The phase-locked loop subsystem generates periodic interrupts so that the LaunchPad LEDs flash at a noticeable rate.
