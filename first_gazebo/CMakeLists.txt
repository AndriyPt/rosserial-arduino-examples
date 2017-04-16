cmake_minimum_required(VERSION 2.8.3)
project(first_gazebo)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS gazebo_ros
)
