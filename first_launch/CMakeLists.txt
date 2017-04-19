cmake_minimum_required(VERSION 2.8.3)
project(first_launch)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS first_gazebo first_control first_description
)
