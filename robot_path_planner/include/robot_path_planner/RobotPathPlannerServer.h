#ifndef ROBOT_PATH_PLANNER_SERVER_H
#define ROBOT_PATH_PLANNER_SERVER_H

#include <robot_path_planner/RobotPathPlanner.h>
#include <robot_path_planner/PointVecToPoseVec.h>

class RobotPathPlannerServer
{
public:
  RobotPathPlannerServer() :
    get_pose_list_server_(nh_.advertiseService(
          "robot_path_planner/get_pose_vec",
          &RobotPathPlannerServer::getPoseVecDiffCallback,
          this))
  {}

private:
  bool getPoseVecDiffCallback(
      robot_path_planner::PointVecToPoseVec::Request &req,
      robot_path_planner::PointVecToPoseVec::Response &res);

private:
  RobotPathPlanner robot_path_planner_;

  ros::NodeHandle nh_;

  ros::ServiceServer get_pose_list_server_;
};

#endif // ROBOT_PATH_PLANNER_SERVER_H

