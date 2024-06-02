cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_project_launch)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  rosserial_python
  turtlebot3_laptop_code
  turtlebot3_gazebo
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
