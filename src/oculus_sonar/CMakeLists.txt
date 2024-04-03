cmake_minimum_required(VERSION 2.8.3)
project(oculus_sonar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/oculus_sonar.cfg
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)