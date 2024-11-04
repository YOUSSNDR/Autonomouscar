# Ros2

Here you will find the ros2 workspace needed for the robot and some info for the robot configuration.

for the the tranform system i will represent the robot (with a simple shape) and the differents sensors equipped on it.

Here are the different position in meter:

Robot from world frame: 1 0 0 0 0 0  "ros2 run tf2_ros static_transform_publisher 1 0 0 0 0 0 world robot"

Sensor1 from Robot: 0 0.0415 0.05 pi 0 0  "ros2 run tf2_ros static_transform_publisher 0 0.0415 0.05 3.14 0 0 robot sensor_1"

Sensor2 from Robot: 0.05 -0.05 0.05 pi/2 0 0  "ros2 run tf2_ros static_transform_publisher 0.05 -0.05 0.05 1.57 0 0 robot sensor_2"

Sensor3 from Robot: -0.05 -0.05 0.05 -pi/2  0 0 "ros2 run tf2_ros static_transform_publisher -0.05 -0.05 0.05 -1.57 0 0 robot sensor_3"

lidar from Robot :  0 0.075 -0.01 0 0 0  "ros2 run tf2_ros static_transform_publisher -0.05 -0.05 -0.01 0 0 0 robot lidar"

To visualise them:
- enter the commands in different terminals 
- run in another terminal "ros2 run rviz2 rviz2"