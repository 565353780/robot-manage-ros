#include <robot_path_planner/RobotPathPlannerServer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RobotPathPlannerServer");

  RobotPathPlannerServer robot_path_planner_server;

  ros::spin();

  return 1;
}

