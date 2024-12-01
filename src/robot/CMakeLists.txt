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

find_library(WIRINGPI_LIB wiringPi)
find_library(PIGPIO_LIB pigpio)

#add_executable(node
#              src/file.cpp
#)
add_executable(encoders 
                #src/main.cpp 
                
                #motors
                src/encoders.cpp 
                src/encoders_publisher.cpp
                
                #rpi4
                #src/rpi4.cpp

                #dwm
                #src/DWM1001_reader.cpp
                #src/raspberry_pico_w_server.py
                )

add_executable(listener
              src/encoders_subscriber.cpp
)

add_executable(robot_controller
              src/robot_controller.cpp
)

add_executable(position_publisher
              src/DWM1001_reader.cpp
)


target_include_directories(encoders PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(listener PUBLIC
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

target_compile_features(encoders PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


 #install(TARGETS 
 # node1
 # node2
 # DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  encoders
  listener
  robot_controller
  position_publisher
  DESTINATION lib/${PROJECT_NAME})

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
target_link_libraries(encoders
  ${catkin_LIBRARIES}
  ${WIRINGPI_LIB}
  ${PIGPIO_LIB}
)

#ament_target_dependencies(node_name
#                         dep1
#                         dep2
#                         etc.
#)
ament_package()
ament_target_dependencies(encoders
                          rclcpp
                          std_msgs
                          geometry_msgs)

ament_target_dependencies(listener
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
                    