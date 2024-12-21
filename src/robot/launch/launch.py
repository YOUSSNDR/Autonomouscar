from launch import LaunchDescription
from launch_ros.actions import Node

#Use ros2 launch package_name launch.py
#In this case, it would be : ros2 launch robot launch.py 

YAML_FILE_PATH = "config/config.yaml"
PACKAGE_NAME = "robot"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=PACKAGE_NAME,
            executable='motor_subscriber',
            name='motor_subscriber',
            parameters=[YAML_FILE_PATH]
        )
    ])