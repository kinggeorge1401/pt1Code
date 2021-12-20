# pt1Code
Code for Bachelor thesis "Entwicklung einer Ortungseinheit für Atemschutzgeräteträger"
--------------------------------------------------------------------------------------
This Code is written for an ESP32 Wroom microcontroller used on a custom designed PCB.

Peripheral devices are:
--------------------------------------------------------------------------------------
- BNO055 IMU 
- Custom 3 Button PCB 
- Adafruit 8 Neopixel LED strip
- 2.4GHz RF Antenna module
- Passive Buzzer

Librarys used:
--------------------------------------------------------------------------------------
- Wire.h library
- Adafruit Sensor lib
- Adafruit BNO055 lib
- ESP32 Servo lib
- SPI library
- RF24 library


Utilized ESP32 Pins
--------------------------------------------------------------------------------------
- Button A - Pin 32
- Button B - Pin 34
- Button Master - Pin 35
- Neopixel LED Strip - Pin 25
- Buzzer - Pin 18
- SCL - Pin 22
- SDA - Pin 21
- MOSI - Pin 23  
- MISO - Pin 19
- CE - 12
- CSS - 5

