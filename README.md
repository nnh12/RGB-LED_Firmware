# RGB-LED_Firmware
Firmware for STM32 Microcontroller firmware using C language. This module indicates our NASA Robot Mining Competition robot's autonomy state using different patterns of the LED stripes.

## main.c
This is the code that takes in the inputed message (CAN Frame message) and filters it out. It then sets it to the appropriate microcontroller counter frequency to turn on the desired color at the certain LED sequence. It also shifts the bits to fit the necessary size by fiting two 8 bit numbers into a 12 bit data array that would be output (array contains LED color information).
