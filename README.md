# Blinky-AngleSensor-Demo-Kit-Software
This is the embedded software for our AG935-07E “Blinky” angle sensor 
demo board.

The demo board uses an Arduino-compatible ATtiny85 microcontroller.
The board comes preprogrammed, so you don't really need the software, 
but we've made it available if you're curious, or in case you want to use 
the routines in your own application.

The demo kit uses an AAT003 Angle Sensor and 60 multicolor smart LEDs 
in a circular array (6° spacing) to show the rotation angle. 
The board also has an analog PWM angle output with a rail-to-rail output 
scaled for zero to 360 degrees. There's also a calibration routine 
that stores parameters in EEPROM.

The software includes ADC routines to read the sensor; digital filtering; 
angle calculation and calibration; and determining motion and direction.

The code was written in the Arduino IDE targeted at "Adafruit Trinket 
(ATtiny85 @ 8 MHz)," and can be ported to other Arduino boards 
(add delays per the program comments for faster processors). 
NeoPixel Arduino routines are used to drive the smart LEDs (NeoPixel arrays use the same type of LEDs).

The program uses ~5 Kbytes out of 8 Kbytes of flash 
(5.4 Kbytes are available with an Arduino bootloader).

For more information on the demo kit or to order online, visit:
https://www.nve.com/webstore/catalog/product_info.php?cPath=27_29&products_id=652
