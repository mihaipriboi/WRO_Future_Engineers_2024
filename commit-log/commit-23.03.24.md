# Commit - 23/03/2024

## Added some basic functions

### commArduino.py
Contains the code that should run on the OpenMV H7 camera in order to communicate with the Arduino Nano ESP32 using the UART communication protocol. This code is used to transmit the speed that the Arduino should give to the motor.

### commOpenMV.ino
Contains the code that should run on the Arduino Nano ESP32 in order to receive the speeds for the motor from the camera and control it using a motor driver. It also has an additional function to get the number of centimeters driven by the motor using an encoder module. An issue came up when working with the Encoder object, since it works only when using pin numbering by GPIO number. If one would use pin numbering by Arduino pin, even though the code compiles and uploads succesfully, it doesn't run, instead the last version uploaded that works will run.