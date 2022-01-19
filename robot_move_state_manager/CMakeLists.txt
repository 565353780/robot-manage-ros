cmake_minimum_required(VERSION 3.0.2)

project(
  robot_move_state_manager
  LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tensorboard_logger_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/RobotMoveStateManager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

