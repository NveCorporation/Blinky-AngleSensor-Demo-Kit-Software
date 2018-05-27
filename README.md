# Blinky-AngleSensor-Demo-Kit-Software
The embedded software for our AG935-07E “Blinky” angle sensor demo board

The demo board uses an Arduino-compatible ATtiny85 microcontroller.
The board comes preprogrammed, so you don't really need the software, 
but we've made it available if you're curious, or in case you want to use 
the routines in your own application.

The demo kit uses an AAT003 Angle Sensor and 60 multicolor smart LEDs 
in a circular array (6° spacing) indicating the rotation angle. 
It also has an analog PWM angle output with a rail-to-rail output scaled 
for zero to 360 degrees. There's also a calibration routing that stores
parameters in EEPROM.

The code was written in Arduino IDE targeted at "Adafruit Trinket 
(ATtiny85 @ 8 MHz)," and can be ported to other Arduino boards 
(add delays per the program comments for faster processors). 
It uses NeoPixel Arduino routines for the smart LEDs for convenience 
(NeoPixel arrays use the same type of LEDs).

The program uses ~5 Kbytes of flash out of 8 Kbytes 
(5.4 Kbytes is available with an Arduino bootloader).
