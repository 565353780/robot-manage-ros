#include <robot_path_planner/RobotPathPlannerServer.h>

bool RobotPathPlannerServer::getNavPoseVecCallback(
    robot_path_planner::TargetPoseVecToNavPoseVec::Request &req,
    robot_path_planner::TargetPoseVecToNavPoseVec::Response &res)
{
  const std::vector<geometry_msgs::Pose>& target_pose_vec =
    req.target_pose_vec;
  const double& delta_rotation_angle = req.delta_rotation_angle;
  const double& delta_move_dist = req.delta_move_dist;

  std::vector<geometry_msgs::Pose> nav_pose_vec;
  if(!robot_path_planner_.getNavPoseVec(
        target_pose_vec,
        delta_rotation_angle,
        delta_move_dist,
        nav_pose_vec))
  {
    std::cout << "[ERROR][RobotPathPlannerServer::getNavPoseVecCallback]\n" <<
      "\t getNavPoseVec failed!\n";
    return false;
  }

  res.nav_pose_vec = nav_pose_vec;

  robot_path_planner::PoseVec nav_pose_vec_copy;
  nav_pose_vec_copy.pose_vec = nav_pose_vec;

  nav_pose_vec_pub_.publish(nav_pose_vec_copy);
  return true;
}

