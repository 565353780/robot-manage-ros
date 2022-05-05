cmake_minimum_required(VERSION 3.0.2)
project(robot_path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  genmsg
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
  src/distance_utils/astar_expander.cpp
  src/distance_utils/dijkstra_expander.cpp
  src/distance_utils/distance_metric.cpp
  src/distance_utils/euclidean_distance.cpp
  src/distance_utils/geodesic_distance.cpp
  src/grid_map/grid_map_2d.cpp
  src/path_generator.cpp
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

