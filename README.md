
# Rover
A repository for a 4-wheel Skid Steering rover, based on Robotic Operative System ROS and Python to control the perception and action systems

## Hardware Requirements:
 - Raspberry pi 3B 
 - RoboteQ Motor Controller HDC2450
 - Xsens MTi-30
 - GPS
 - Encoders
 - Control RC SPEKTRUM DX6i
 
### Software Requirements
  - [Ubiquity OS](https://www.ubiquityrobotics.com/)
  - [pigpio](http://abyz.me.uk/rpi/pigpio/python.html)
  - [Python 2.7](https://www.python.org/download/releases/2.7/)
  - [numpy](https://www.numpy.org/)

## Raspberry pi 
In a terminal of Ubiquity, first create a catkin workspace folder

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
After create the workspace folder, in a /src folder download the folders of this repository [Rover_ROS](https://github.com/JuanCarlos-TiqueRangel/Rover/tree/master/Rover_ROS) 
that contain all the necesary elements to run the autonomous navigation of the Rover

```sh
sudo apt-get install htop
sudo apt-get install screen
```


***
### Author:
**Universidad de Ibagué** - **Ingeniería Electrónica**

  - [Juan Carlos Tique](https://github.com/JuanCarlos-TiqueRangel)
***



