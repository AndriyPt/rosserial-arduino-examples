# rosserial-arduino-examples
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

Go to *~/workspace/src* folder on Docker container.
Clone repository
```bash
git clone https://github.com/AndriyPt/rosserial-arduino-examples.git
```
Got to *~/workspace*.
Install dependencies.
```bash
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

Compile source code
```bash
catkin_make
```
and
```bash
source ~/workspace/devel/setup.bash
```

### Install Arduino
Download and extract Arduino IDE in home folder.

### Setup ros_serial_arduino
In Arduino's libraries folder run
```bash
source ~/workspace/devel/setup.bash
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```
More details can be found [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).

## Robot model preview
In order to check robot's URDF and move it's joint run
```bash
roslaunch first_description display.launch gui:=True
```

## Simulation
```bash
roslaunch first_launch simulation.launch
```

## Real Hardware 
```bash
roslaunch first_launch real_robot.launch
```

## Tools
Different tools are available in RQT
```bash
rqt
```