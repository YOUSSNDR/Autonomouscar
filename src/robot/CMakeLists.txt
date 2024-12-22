cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#Message packages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_library(PIGPIO_LIB pigpio)


#testing with PIGPIO_IF2 lib instead of pigpio
find_library(PIGPIO_IF2_LIB pigpiod_if2)

#add_executable(node
#              src/file.cpp
#)

#home/youss/Documents/Projet/Autonomouscar/ros_ws/install/robot/lib/robot/encoders: 
#error while loading shared libraries: liblibstatistics_collector.so: cannot open shared object file: No such file or directory
#[ros2run]: Process exited with failure 127
#To fix the bug, disable src/encoders_publisher.cpp

#Add every cpp file needed in a node

add_executable(main 
                src/main.cpp 
                
                #motors
                src/motor.cpp

                #encoders
                src/encoders.cpp 
                #src/encoders_publisher.cpp
                
                #rpi4
                src/rpi4.cpp


                #src/raspberry_pico_w_server.py
                )

add_executable(encoders_subscriber
              src/encoders_subscriber.cpp
)

add_executable(encoders_publisher
              src/encoders_publisher.cpp
              src/encoders.cpp
)

add_executable(robot_controller
              src/robot_controller.cpp
)

add_executable(position_publisher
              src/DWM1001_reader.cpp
)

add_executable(motor_subscriber
              src/motor_subscriber.cpp
)

add_executable(position_reader
              src/DWM1001_reader.cpp)

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(encoders_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


 
  
# fatal error: robot/robot_controller.h: No such file or directory
  #  1 | #include "robot/robot_controller.h"

#add this line otherwise the error above occurs 
target_include_directories(robot_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(position_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(motor_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(position_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(encoders_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


 #install(TARGETS 
 # node1
 # node2
 # DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  main
  encoders_subscriber
  robot_controller
  position_publisher
  position_reader
  motor_subscriber
  encoders_publisher
  DESTINATION lib/${PROJECT_NAME})


#This command ensures the launch/ directory is installed in the share directory of the package.
#Don't forget to add <test_depend>launch</test_depend> in the package.xml file
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

target_link_libraries(main
  ${catkin_LIBRARIES}
  ${PIGPIO_LIB}
  ${PIGPIO_IF2_LIB}

)


#If you want to use motor.cpp, add this or it will not work
target_link_libraries(motor_subscriber
  ${catkin_LIBRARIES}
  ${PIGPIO_LIB}
  ${PIGPIO_IF2_LIB}
)


#Needed to use the pigpio library in the encoders_publisher node
target_link_libraries(encoders_publisher
  ${catkin_LIBRARIES}
  ${PIGPIO_LIB}
  ${PIGPIO_IF2_LIB}

)

#ament_target_dependencies(node_name
#                         rosdep1
#                         rosdep2
#                         etc.
#)
ament_package()
ament_target_dependencies(main
                          rclcpp
                          std_msgs
                          geometry_msgs)

ament_target_dependencies(encoders_subscriber
                          rclcpp
                          std_msgs)

ament_target_dependencies(robot_controller
                          rclcpp
                          std_msgs
                          geometry_msgs)

ament_target_dependencies(position_publisher
                          rclcpp
                          std_msgs
                          geometry_msgs)

ament_target_dependencies(position_reader
                          rclcpp
                          std_msgs
                          geometry_msgs)

ament_target_dependencies(motor_subscriber
                          rclcpp
                          std_msgs
                          geometry_msgs)

ament_target_dependencies(encoders_publisher
                          rclcpp
                          std_msgs
                          geometry_msgs)