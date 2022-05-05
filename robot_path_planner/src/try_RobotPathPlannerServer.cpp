#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

#include <robot_path_planner/TargetPoseVecToNavPoseVec.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "try_RobotPathPlannerServer");

  ros::NodeHandle nh;
  ros::ServiceClient try_get_nav_pose_vec_client =
    nh.serviceClient<robot_path_planner::TargetPoseVecToNavPoseVec>(
        "robot_path_planner/get_nav_pose_vec");

  std::cout << "Start call robot_path_planner_server service...\n";

  std::cout << "Start wait robot_path_planner_server...\n";
  std::vector<geometry_msgs::Pose> target_pose_vec;
  geometry_msgs::Pose new_pose;
  new_pose.orientation.x = 0;
  new_pose.orientation.y = 0;
  new_pose.orientation.z = 0;
  new_pose.orientation.w = 1;

  new_pose.position.x = 0;
  new_pose.position.y = 0;
  new_pose.position.z = 1;
  target_pose_vec.emplace_back(new_pose);
  new_pose.position.x = 1;
  new_pose.position.y = 0;
  new_pose.position.z = 1;
  target_pose_vec.emplace_back(new_pose);
  new_pose.position.x = 1;
  new_pose.position.y = 1;
  new_pose.position.z = 1;
  target_pose_vec.emplace_back(new_pose);

  const double delta_rotation_angle = M_PI / 15 / 30;
  const double delta_move_dist = 0.15 / 30;
  robot_path_planner::TargetPoseVecToNavPoseVec get_nav_pose_vec_serve;
  get_nav_pose_vec_serve.request.target_pose_vec = target_pose_vec;
  get_nav_pose_vec_serve.request.delta_rotation_angle = delta_rotation_angle;
  get_nav_pose_vec_serve.request.delta_move_dist = delta_move_dist;

  if(!try_get_nav_pose_vec_client.call(get_nav_pose_vec_serve))
  {
    std::cout << "[ERROR][try_RobotPathPlannerServer::main]\n" <<
      "\t call robot_path_planner/get_nav_pose_vec failed!\n";
    return -1;
  }

  const std::vector<geometry_msgs::Pose>& nav_pose_vec =
    get_nav_pose_vec_serve.response.nav_pose_vec;

  std::cout << "[INFO][try_RobotPathPlannerServer::main]\n" <<
    "\t target_pose_vec.size() = " << target_pose_vec.size() << std::endl <<
    "\t nav_pose_vec.size() = " << nav_pose_vec.size() << std::endl;

  return 1;
}

