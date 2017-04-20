cmake_minimum_required(VERSION 2.8.3)
project(first_hardware)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  std_msgs
  first_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES first_hardware
   CATKIN_DEPENDS hardware_interface controller_manager roscpp std_msgs first_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/first_robot_hardware.cpp src/hardware_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES robot_hardware_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)