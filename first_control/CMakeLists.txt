cmake_minimum_required(VERSION 2.8.3)
project(first_control)

find_package(catkin REQUIRED COMPONENTS
  ros_control
  ros_controllers
)

catkin_package(
  CATKIN_DEPENDS ros_control ros_controllers
)
