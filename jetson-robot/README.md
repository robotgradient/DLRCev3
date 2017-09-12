This repository contains some basic functionality for the Sensors and Motors. 
You will use Jetson TX2 and lego mindstorm ev3. 

The ev3 has been flashed with ev3dev (on microSD), see http://www.ev3dev.org/docs/getting-started/

Sensor and Actuator classes are implemented for the communication between Jetson and Ev3.
Furthermore, a python wrapper for IMU Sensor and camera is already implemented.
You can use these as a quick-starter.

To build the shared library for MPU6050 sensor you can use:
gcc -fPIC -shared mpu6050.c -o libmpu.so
If you get error "Failed to open i2c port", you could do (although a bit hacky): sudo chmod 666 /dev/i2c-1



In IOInterface, the folder ev3 will be deployed on the ev3 and folder jetson is needed on Jtson TX2.
Clone this repo on your Jetson and also on the ev3 (or just scp). To clone it onto ev3, we need to set up a few things first:

- Set up a connection between Jetson and ev3, following http://www.ev3dev.org/docs/tutorials/connecting-to-the-internet-via-usb/
- ssh into ev3 (ssh robot@ev3dev.local)

- git clone http://gitlab.argmax.ai/dlrc/jetson-robot
- In ev3/config.py and jetson/config.py adapt BROKER_IP to the ip address of your Jetson (ifconfig).
- Follow "Requirements" in http://www.ev3dev.org/docs/tutorials/sending-and-receiving-messages-with-mqtt/ and also install mosquitto on Jetson (sudo apt-get install mosquitto)

Start program on ev3:
python3 main.py
