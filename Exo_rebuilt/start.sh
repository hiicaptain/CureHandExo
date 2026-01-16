#!/bin/bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sleep 1
gnome-terminal -- sh -c "roslaunch dynamixel.launch;exec bash" 
sleep 5
gnome-terminal -- sh -c "roslaunch meta_controller.launch;exec bash" 
sleep 5
gnome-terminal -- sh -c "python3 sensor_value.py;exec bash"
sleep 5
gnome-terminal -- sh -c "python3 motor_control_noemg.py;exec bash"
sleep 5
gnome-terminal -- sh -c "python3 experiment.py;exec bash"
printf "GUI is running\n"

