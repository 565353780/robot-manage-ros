#ifndef ROBOT_PATH_PLANNER_SERVER_H
#define ROBOT_PATH_PLANNER_SERVER_H

#include <robot_path_planner/RobotPathPlanner.h>
#include <robot_path_planner/PoseVec.h>
#include <robot_path_planner/TargetPoseVecToNavPoseVec.h>

class RobotPathPlannerServer
{
public:
  RobotPathPlannerServer() :
    get_pose_list_server_(
        nh_.advertiseService(
          "robot_path_planner/get_nav_pose_vec",
          &RobotPathPlannerServer::getNavPoseVecCallback,
          this)),
    nav_pose_vec_pub_(
        nh_.advertise<robot_path_planner::PoseVec>(
          "robot_path_planner/nav_pose_vec", queue_size_))
  {
    queue_size_ = 1;
  }

private:
  bool getNavPoseVecCallback(
      robot_path_planner::TargetPoseVecToNavPoseVec::Request &req,
      robot_path_planner::TargetPoseVecToNavPoseVec::Response &res);

private:
  RobotPathPlanner robot_path_planner_;

  ros::NodeHandle nh_;
  size_t queue_size_;

  ros::ServiceServer get_pose_list_server_;
  ros::Publisher nav_pose_vec_pub_;
};

#endif // ROBOT_PATH_PLANNER_SERVER_H

