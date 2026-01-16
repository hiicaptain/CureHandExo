# CureHandExo
This is the control software for my phd project on a hand exoskeleton

# Installation
## 1. install ROS Noetic
https://wiki.ros.org/noetic/Installation/Ubuntu

## 2. clone this repository
```
python3 -m venv .venv
py
git clone --recursive https://github.com/hiicaptain/CureHandExo.git
#deactivate
```

## 3.1 install dynamixel Workbench for ros demo
https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/

### download the ros packages into src in the root
```
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

### build the ros packages to 
initialise the workspace, you probably need to install Eigen3 library and yaml-cpp package
```
cmake
sudo apt-get update
sudo apt-get install libeigen3-dev
sudo apt-get install libyaml-cpp-dev
```

source the workspace
```
source devel/setup.bash
```

To avoid having to source the setup file in every new terminal session manually, you can add the source command to your .bashrc file:
```
echo "source ${YOUR_WORKSPACE_PATH}/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### test
```
rosrun dynamixel_workbench_controllers find_dynamixel /dev/ttyUSB0
```

## 3.2 install dynamixel Workbench for cpp demo codes
### setup DYNAMIXEL SDK library
http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#build-the-library

### setup DYNAMIXEL Workbench toolbox library
```
cd ${YOUR_DOWNLOAD_PATH}/dynamixel_workbench/dynamixel_workbench_toolbox/examples
mkdir build && cd build
cmake ..
make
sudo chmod a+rw /dev/ttyUSB0
./find_dynamixel /dev/ttyUSB0
```

# Debug
You may need to change the port names in 'dynamixel.launch' and 'sensor_vale.py' according to the serial ports that you connect to dynamixel servos and arduino board.

This run file configures the usb latency timer to 1 ms. If you want to check this setting, run the following command in a terminal window.
```
# check the latency time
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# set the latency time to 1 ms
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

You may also need to set the serial port readable. Please remember to change the correct port name.
```
sudo chmod 777 /dev/ttyUSB0
```

# Running
There are two ways to run the exoskeleton control system: using the automated startup script or launching components manually.

## Method 1: Automatic Startup
From the following directory: CureHandExo/Exo_rebuilt/, run: 
```
./start.sh. 
```
This script automatically launches all required ROS nodes, background processes, and the GUI.

## Method 2: Manual Launch
Open separate terminals and run the following commands in order:
```
# Launch the Dynamixel ROS controller (automatically detects the four motors):
roslaunch dynamixel.launch
# Launch the meta controller containing motor configuration parameters:
roslaunch meta_controller.launch
# Start the Arduino sensor interface, which records angle and force data and publishes them as ROS topics (ensure the correct port is selected):
python3 sensor_value.py
# Run the motor control backend, responsible for reading motor states and forwarding control commands:
python3 motor_control_noemg.py
# Launch the experimental GUI:
python3 experiment.py
```



