#ifndef ROBOT_PATH_PLANNER_H
#define ROBOT_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

class RobotPathPlanner
{
public:
  RobotPathPlanner()
  {}

  bool getNavPoseVec(
      const std::vector<geometry_msgs::Pose>& pose_vec,
      const double& delta_rotation_angle,
      const double& delta_move_dist,
      std::vector<geometry_msgs::Pose>& nav_pose_vec);

private:
  bool updateParam(
      const double& delta_rotation_angle,
      const double& delta_move_dist);

  double getPointNorm2(
      const geometry_msgs::Point& point);
  double getPointNorm(
      const geometry_msgs::Point& point);

  bool getUnitPoint(
      const geometry_msgs::Point& source_point,
      geometry_msgs::Point& unit_point);

  bool getUnitAngle(
      const double& source_angle,
      double& unit_angle);

  bool getPoseFaceToAngle(
      const geometry_msgs::Pose& pose,
      double& face_to_angle);

  bool getPoseFaceToDirection(
      const geometry_msgs::Pose& pose,
      geometry_msgs::Point& face_to_direction);

  bool getRotatePoseVec(
      const geometry_msgs::Pose& source_pose,
      const geometry_msgs::Point& target_direction,
      std::vector<geometry_msgs::Pose>& rotate_pose_vec);

  bool getMovePoseVec(
      const geometry_msgs::Pose& source_pose,
      const geometry_msgs::Point& target_position,
      std::vector<geometry_msgs::Pose>& move_pose_vec);

  bool getSingleNavPoseVec(
      const geometry_msgs::Pose& source_pose,
      const geometry_msgs::Pose& target_pose,
      const bool& need_face_to_target_pose,
      std::vector<geometry_msgs::Pose>& nav_pose_vec);

private:
  double delta_rotation_angle_;
  double delta_move_dist_;

  tf2::Quaternion delta_rotation_quat_;
  tf2::Quaternion delta_opposite_rotation_quat_;
};

#endif // ROBOT_PATH_PLANNER_H

