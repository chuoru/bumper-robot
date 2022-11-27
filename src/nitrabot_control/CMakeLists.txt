cmake_minimum_required(VERSION 3.0.2)
project(scart_mock_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
  geometry_msgs nav_msgs std_msgs tf message_generation
  rostest
  )

catkin_python_setup()

add_message_files(
  FILES
  RouteDataMsg.msg
  MarkerDataMsg.msg
)

add_service_files(
  FILES
  GetRoute.srv
  SetRoute.srv
  StartRoute.srv
  CourseReset.srv
  EmergencyStop.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs nav_msgs std_msgs tf message_runtime dynamic_reconfigure
  )


catkin_install_python(PROGRAMS scripts/scart_mock_controller_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
