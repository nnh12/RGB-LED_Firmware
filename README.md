# RGB-LED_Firmware
Firmware for STM32 Microcontroller firmware using C language. This module indicates our NASA Robot Mining Competition robot's autonomy state using different patterns of the LED stripes.

## main.c
This is the code that takes in the inputed message (CAN Frame message) and sets it to the appropriate microcontroller counter frequency to turn on the desried color. It also shifts the bits to fit the necessary size (manages to fit the 8 bit into a 12 bit data array).
