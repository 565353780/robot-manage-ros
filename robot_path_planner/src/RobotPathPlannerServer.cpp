#include <robot_path_planner/RobotPathPlannerServer.h>

bool RobotPathPlannerServer::getPoseVecDiffCallback(
    robot_path_planner::PointVecToPoseVec::Request &req,
    robot_path_planner::PointVecToPoseVec::Response &res)
{
  return true;
}

