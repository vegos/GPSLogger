GPSLogger -- (c)2012, Antonis Maglaras

GPSLogger logs GPS information in combination with acceleration data.
It started as a project for my RC Bike, to count the max leaning angles
and acceleration/forces on X/Y axes.

GPSLogger logs the following data on a CSV file:
Time, Date (from GPS, in UTC)
Latitude, Logitude
Altitude
Speed (It counts speeds bigger than 0.4m/s (or 1.4km/h)
Course (heading)
Acceleration on X axis
Acceleration on Y axis
Acceleration on Z axis
Angle on X axis
Angle on Y axis

The update rate is @ 1Hz (once per second). 
Acceleration data are received realtime, and between writting the data
on the SD card, the max values (of the second) are kept for this.

The project consists of:
Arduino Nano v3
Fastrax UP501 GPS
SD Card module
ADXL345 3-axis Acceleration Module

Also for the hardware part, a custom board with a Voltage Regulator & a 
diode for reverse polarity protection, 3 leds (SD Card Status, GPS Update
Rate, GPS Fix), one Button (for save/close the file) and a Buzzer.


You can find Some photos of the project here:
https://picasaweb.google.com/104656736936976952947/AcceleroTemperatureGPSLoggerV101Beta







GPSLogger needs these libraries:

SoftwareSerial - http://arduino.cc/en/Reference/SoftwareSerial
TinyGPS - http://arduiniana.org/libraries/tinygps/
SD - http://www.arduino.cc/en/Reference/SD
Wire - http://www.arduino.cc/en/Reference/Wire
ADXL345 - https://github.com/jenschr/Arduino-libraries
