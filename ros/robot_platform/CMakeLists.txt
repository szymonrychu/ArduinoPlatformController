cmake_minimum_required(VERSION 2.8.3)
project(robot_platform)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  MotorStatus.msg
  ServoStatus.msg
  PlatformStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs message_runtime message_generation
)

catkin_install_python(
  PROGRAMS nodes/platform_controller nodes/joy_controller nodes/gui_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
