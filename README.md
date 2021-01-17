# UCLA-IEEE-Hackathon
Home security System
This project involves connecting an ESP-32 board and an ESP-32 camera module to each other using WiFi.

#Board 1 (ESP-32)
This board is used to control the servo motor in order to rotate the camera module.
Parts:
 - OLED 128x64 display
 - Potentiometer
 - RGB LED

#Board 2 (camera module)
This board will take readings from a PIR sensor to activate the OLED screen from rest mode, as well as change the RGB LED color from green(inactive), to red(motion detected).
Parts:
 - Servo motor
 - PIR sensor
 
