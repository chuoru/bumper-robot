cmake_minimum_required(VERSION 3.0.2)
project(nitrabot_control)

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

catkin_install_python(PROGRAMS scripts/bumper_control_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
