
# Rover
A repository for a 4-wheel Skid Steering rover, based on Robotic Operative System ROS and Python to control the perception and action systems for autonomous navigation. Using Model Predictive Controller for follow the reference angle, Proportional error for speed and Kalman Filter to stimate the x,y position. 

<p align='center'>
    <img src="./config/doc/rover_trajectory.gif" alt="drawing" width="600"/>
</p>

<p align='center'>
    <img src="./config/doc/rover_1.jpg" alt="200" width="200"/>
    <img src="./config/doc/rover_1.gif" alt="drawing" width="200"/>
    <img src="./config/doc/rover_2.jpg" alt="drawing" width="200"/>
    <img src="./config/doc/rover_2.gif" alt="drawing" width="200"/>
</p>

 - folder with all ROS nodes [Rover_ROS](Rover_ROS)
 - folder with codes from previous jobs [Rover_resource](Rover_resource)

## Hardware Requirements:
 - [Raspberry pi 3B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) 
 - [RoboteQ Motor Controller HDC2450](config/doc/hdc24xx_datasheet.pdf)
 - [Xsens MTi-30](config/doc/MTi_usermanual.pdf)
 - [GPS Ublox 3DR](https://www.amazon.es/3DR-uBlox-GPS-Compass-Kit/dp/B00FNPAD4K)
 - [Encoders Autonics E50s-1500](config/doc/E50S.pdf)
 - [Control RC SPEKTRUM DX6i](config/doc/SPM6600-Manual_DX6i.pdf)
 - [Receptor SPEKTRUM AR8000](config/doc/SPMAR8000-Manual.pdf)
 
## Software Requirements
  - [Ubiquity OS](https://www.ubiquityrobotics.com/)
  - [pigpio](http://abyz.me.uk/rpi/pigpio/python.html)
  - [Python 2.7](https://www.python.org/download/releases/2.7/)
  - [numpy](https://www.numpy.org/)

## Raspberry PI 
- First download [Ubiquity_OS](https://www.ubiquityrobotics.com/) in a micro SD preferably type 10 for a good performance 
- Insert the micro SD in a Raspberry pi 
- Open a Terminal with Ctrl + T and write:
```
sudo raspi-config
```
<p align='center'>
    <img src="./config/doc/raspi-config.png" alt="drawing" width="600"/>
</p>

- In Boot options disable the GUI or graphical interface, this is the best option to get the optimal performance in a Raspberry Pi. When the raspberry restart only have the comand window. 

In a terminal of Ubiquity, first create a catkin workspace folder

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
After create the workspace folder, in a /src folder download the folders of this repository [Rover_ROS](Rover_ROS/) 
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
```
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/ethzasl_xsens_driver.git
cd ~/catkin_ws/
catkin_make
```
Xsens company now create his own node in ROS, to download them, first need download the [MT Software Suit](https://www.xsens.com/software-downloads) for linux and follow the instructions into a readme files. 

## Udev Rules
Udev is a device manager for Linux that dynamically creates and removes nodes for hardware devices. In short, it helps your computer find your robot easily. By default, hardware devices attached to your Linux (Ubuntu) PC will belong to the root user. This means that any programs (e.g. ROS nodes) running as an unpriveleged (i.e. not root) user will not be able to access them. On top of that, devices will receive names such as ttyACMx and ttyUSBx arbitrarily based on the order in which they were plugged in. Luckily, you can solve this, and more, with udev rules in the web site of [CLEARPATH](http://www.clearpathrobotics.com/assets/guides/kinetic/ros/Udev%20Rules.html). implement udev is used to identify the sensors or peripherals that are connected through the Raspeberry USB ports. 

Some driver/software packages will already provide udev rules you can use. Check the /etc/udev/rules.d/ folder to see if there’s anything installed already. If the package is lazy and gives you a udev rule to install yourself, you can do this using

```
sudo cp <rule file> /etc/udev/rules.d/>
```

### Matching
The matching part lets the udev device manager match the rule to the device you want. The manager will try to match all new devices as they get plugged in, so it’s important that the rule be specific enough to capture only the device you’re looking for, otherwise you’ll end up with a /dev/hokuyo symlink to an IMU. There are many potential matching tags, and the best way to pick the useful ones is to get all the device attributes straight from udev.

Run the following cammand, inserting a <devpath> such as /dev/ttyACM0:
```
udevadm info -a -p $(udevadm info -q path -n <devpath>)
```
You will get a list of all device attributes visible to udev. looking at device /dev/ttyACM0
```sh
KERNEL=="ttyACM0"
SUBSYSTEM=="tty"
DRIVER==""
looking at parent device '...':
KERNELS=="3-3:1.0"
SUBSYSTEMS=="usb"
DRIVERS=="cdc_acm"
ATTRS{bInterfaceClass}=="02"
ATTRS{bInterfaceNumber}=="00"
looking at parent device '...':
...
ATTRS{idVendor}=="0483"
ATTRS{idProduct}=="5740"
... 
```
    

## Functios to add on Raspberry in a .bashrc file

- The next function it's for move the Rover to place that want to do Autonomous navigation
```
function manual(){
 /usr/bin/python /catkin_ws/src/rc_control/src/control.py &
 sleep 10

 /usr/bin/python /catkin_ws/src/main_control/src/mpc_controller.py 
 sleep 10
 }
```
And this one it's for execute the algorithms necessaries for trajectory tracking. 

```
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

## System node map
The next picture is a map of all the nodes running on the system in automatic mode

<p align='center'>
    <img src="./config/doc/mapa_nodos.png" alt="drawing" width="600"/>
</p>

If you found this work usefull and help in your research, please cite our work, Thank you!

```tex
@INPROCEEDINGS{9633291,
  author={Barrero, Oscar and Tique, Juan C.},
  booktitle={2021 IEEE 5th Colombian Conference on Automatic Control (CCAC)}, 
  title={MBPC controller for UGV Trajectory Tracking}, 
  year={2021},
  pages={314-319},
  doi={10.1109/CCAC51819.2021.9633291}}
```


***
### Author:
**Universidad de Ibagué** - **Ingeniería Electrónica**

  - [Juan Carlos Tique](https://github.com/JuanCarlos-TiqueRangel)
***



