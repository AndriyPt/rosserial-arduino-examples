# ros-meetup-examples
## Hardware 
  * [FC-03](https://www.aliexpress.com/item/Tacho-sensor-Slot-type-Optocoupler-Tacho-generator-Counter-Module-for-Arduino-for-Raspberry-pi/32319062749.html) 
  * Toy DC motor

## Arduino and wiring
I have used Arduino Mega 2560.

Pin 2 was connected to FC-03 digital output.
5V and GND was connected to appropriate inputs of FC-03.

Motor was connected using this circuit:
![Motor Wiring](https://www.codeproject.com/KB/boards-embedded-devices/845211/9.1.jpg "Motor Wiring")

## Docker Container
For development [the following](https://hub.docker.com/r/shadowrobot/build-tools/) docker container was used.
To pull it please run
```bash
docker pull shadowrobot/build-tools:xenial-kinetic-ide
```
If you want to run everything from docker container use the following command
```bash
docker run -it --name meetup_demo --privileged -e DISPLAY  -e LOCAL_USER_ID=$(id -u) -v /dev/ttyUSB0:/dev/ttyUSB0 -v /tmp/.X11-unix:/tmp/.X11-unix:rw shadowrobot/build-tools:xenial-kinetic-ide
```

## Setup

### Workspace installation
### Install Arduino
### Setup ros_serial_arduino

## Simulation
```bash
roslaunch first_launch simulation.launch
```

## Real Hardware 
```bash
roslaunch first_launch real_robot.launch
```
