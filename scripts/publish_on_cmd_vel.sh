#!/bin/bash

# Check if enough arguments are provided
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <linear_x> <angular_z>"
  exit 1
fi

# Assign arguments to variables
LINEAR_X=$1
ANGULAR_Z=$2

# Publish to the /cmd_vel topic
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: $LINEAR_X, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_Z}}"
