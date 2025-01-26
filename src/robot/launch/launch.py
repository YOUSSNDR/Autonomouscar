from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_PACKAGE = "robot"
ROBOT_NAMESPACE = "robot"

#Use emulate_tty=True to display rclcpp_error in red

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=ROBOT_PACKAGE,
            namespace=ROBOT_NAMESPACE,
            executable="encoders_subscriber",
            emulate_tty=True
        ),
        Node(
            package=ROBOT_PACKAGE,
            namespace=ROBOT_NAMESPACE,
            executable="robot_controller",
            emulate_tty=True
        ),
        Node(
            package=ROBOT_PACKAGE,
            namespace=ROBOT_NAMESPACE,
            executable="position_reader",
            emulate_tty=True
        ),
        Node(
            package=ROBOT_PACKAGE,
            namespace=ROBOT_NAMESPACE,
            executable="motor_subscriber",
            emulate_tty=True
        ),
        Node(
            package=ROBOT_PACKAGE,
            namespace=ROBOT_NAMESPACE,
            executable="encoders_publisher",
            emulate_tty=True
        ),
        Node(
            package=ROBOT_PACKAGE,
            namespace=ROBOT_NAMESPACE,
            executable="mpu_publisher",
            emulate_tty=True
        ),
    ])