#!/bin/bash

gnome-terminal -- sh -c "roslaunch dynamixel.launch;exec bash" 
sleep 5
gnome-terminal -- sh -c "roslaunch meta_controller.launch;exec bash" 
sleep 5
gnome-terminal -- sh -c "python3 sensor_value.py;exec bash"
sleep 7
gnome-terminal -- sh -c "python3 experiment.py;exec bash"
printf "GUI is running\n"

