cmake_minimum_required(VERSION 2.8.3)
project(first_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  message_generation
  std_msgs
)

add_service_files(
  FILES
  GetMotorSpeed.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
