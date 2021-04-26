# SensorMonitor
PyQt app to monitor the sensor data from the 10DOF SEN0140 board as well as my own magnetic field RGB test data. This app receives sensor data from an arduino nano over a serial connection. The magnetic field data is corrected for both soft and hard iron deposits and the compass heading is tilt compensated.

RGB Test modes:  
Mode 1) 3-axis lightshow. The red, green, and blue LEDs will light up depending on the magnetic field intensity in the X, Y, and Z axis respectively.   

Mode 2) Polarity mode. The red and blue LEDs show the north or south polarity of the magnetic field on the Z axis.  
  
Holding the device up to a fan or other moving magnetic field produces a very cool lightshow in either mode.

![SensorMonitor_V1](https://user-images.githubusercontent.com/10580033/113110450-f299a700-91bb-11eb-9b13-60a44e38f393.png)

Breadboard layout:  
![breadboard_layout](https://user-images.githubusercontent.com/10580033/113110452-f3323d80-91bb-11eb-83c3-0d74dca7cfb2.jpeg)


