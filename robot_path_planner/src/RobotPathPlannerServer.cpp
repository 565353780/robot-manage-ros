#include <robot_path_planner/RobotPathPlannerServer.h>

bool RobotPathPlannerServer::getNavPoseVecDiffCallback(
    robot_path_planner::TargetPoseVecToNavPoseVec::Request &req,
    robot_path_planner::TargetPoseVecToNavPoseVec::Response &res)
{
  return true;
}

