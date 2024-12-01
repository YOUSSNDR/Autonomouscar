#!/bin/bash

compiled_file_name="encoders"
path=/home/youss/Documents/Projet/Autonomouscar/ros_ws/install/robot/lib/robot
total_path=$path/$compiled_file_name 
sudo chown root:root $total_path
sudo chmod 4755 $total_path

echo make_encoders_work - Success on $total_path

