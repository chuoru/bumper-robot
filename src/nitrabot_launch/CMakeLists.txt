cmake_minimum_required(VERSION 3.0.2)
project(nitrabot_launch)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs std_msgs tf message_generation
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs tf message_runtime
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})