cmake_minimum_required(VERSION 3.0.2)
project(robot_path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  tf2
  tf2_geometry_msgs
  cv_bridge
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  TargetPoseVecToNavPoseVec.srv
)

add_message_files(
  FILES
  PoseVec.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
  src/RobotPathPlanner.cpp
  src/RobotPathPlannerServer.cpp
  src/RobotPathPlannerServer_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${catkin_LIBRARIES}
)

target_compile_features(${PROJECT_NAME}_node
  PRIVATE
    cxx_std_17
)

add_executable(try_${PROJECT_NAME}
  src/try_RobotPathPlannerServer.cpp
)

target_link_libraries(try_${PROJECT_NAME} ${catkin_LIBRARIES})

target_compile_features(try_${PROJECT_NAME}
  PRIVATE
    cxx_std_17
)

