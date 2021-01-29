
# Rover
A repository for a 4-wheel Skid Steering rover, based on Robotic Operative System ROS and Python to control the perception and action systems.

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

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
After create the workspace folder, in a /src folder download the folders of this repository [Rover_ROS](https://github.com/JuanCarlos-TiqueRangel/Rover/tree/master/Rover_ROS) 
that contain all the necesary elements to run the autonomous navigation of the Rover

```sh
sudo apt-get install htop
sudo apt-get install screen
```
## Install repositories

- Install gps_common as available based on the ROS distributable
```sh
sudo apt-get install ros-kinetic-gps-common
```
- Install MTi rosnode
```sh
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/ethzasl_xsens_driver.git
cd ~/catkin_ws/
catkin_make
```
Xsens company now create his own node in ROS, to download them, first need download the [MT Software Suit](https://www.xsens.com/software-downloads) and follow the instructions into a readme files. 

## Functios to add on Raspberry in a .bashrc file

```
function manual(){
 /usr/bin/python /catkin_ws/src/rc_control/src/control.py &
 sleep 10

 /usr/bin/python /catkin_ws/src/main_control/src/mpc_corregido.py 
 sleep 10
 }

function automatico(){
/usr/bin/python /catkin_ws/src/imu/src/imu.py &
sleep 10

/usr/bin/python /catkin_ws/src/gps/src/gps.py &
sleep 10

/usr/bin/python /catkin_ws/src/Roboteq/src/read_enc.py &
sleep 10

/usr/bin/python /catkin_ws/src/Roboteq/src/pos.py &
sleep 10

/usr/bin/python /catkin_ws/src/kalman_filter/src/kalman_bias.py &
sleep 10
}
```



***
### Author:
**Universidad de Ibagué** - **Ingeniería Electrónica**

  - [Juan Carlos Tique](https://github.com/JuanCarlos-TiqueRangel)
***



