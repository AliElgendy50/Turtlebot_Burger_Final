cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_laptop_code)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/laptop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
