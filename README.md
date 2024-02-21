# CureHandExo
This is the control software for my phd project on a hand exoskeleton

# Installation
## 1. install ROS Noetic
https://wiki.ros.org/noetic/Installation/Ubuntu

## 2. clone this repository
'''
python3 -m venv .venv
source .venv/bin/activate
git clone --recursive https://github.com/hiicaptain/CureHandExo.git
# deactivate
'''

## 3. install dynamixel Workbench
https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/

### 3.1 download the ros package
'''
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
'''

### 3. -setup DYNAMIXEL SDK library
http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#build-the-library

### 3. -setup DYNAMIXEL Workbench toolbox library
```
$ cd ${YOUR_DOWNLOAD_PATH}/dynamixel_workbench/dynamixel_workbench_toolbox/examples
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo chmod a+rw /dev/ttyUSB0
$ ./find_dynamixel /dev/ttyUSB0
```

# Usage
This run file configures the usb latency timer to 1 ms. If you want to check this setting, run the following command in a terminal window.
'''
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
'''


