# Temperature-Monitor

# Platform:

This software works on the TI EK-TM4C123GXL Microcontroller.

# Description:

This program reads the temperatures measured by the heat sensor and streams the readings onto the UART connection which are then displayed on the screen. The flash rate of the LED varies as a function of temperature.

# Implementation:

This program configures an analog to digital convertor (ADC0) to read from the internal temperature sensor on the Tiva Launchpad Development board. The raw data is then converted to Celsius temperature and transferred over UART0 at 9600.
We used the SysTick subsystem to generate periodic interrupts so that we can turn on and off one of the LaunchPad LEDs on the development board at a noticeable rate.

# Testing:

We used a heat gun to heat up the board. The LED should start flashing at a higher rate when the chip is hot, and then slow down as it cools off.
