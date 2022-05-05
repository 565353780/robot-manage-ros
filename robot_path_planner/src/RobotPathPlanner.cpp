#include <robot_path_planner/RobotPathPlanner.h>

bool RobotPathPlanner::getNavPoseVec(
    const std::vector<geometry_msgs::Pose>& pose_vec,
    const double& delta_rotation_angle,
    const double& delta_move_dist,
    std::vector<geometry_msgs::Pose>& nav_pose_vec)
{
  nav_pose_vec.clear();

  if(!updateParam(delta_rotation_angle, delta_move_dist))
  {
    std::cout << "[ERROR][RobotPathPlanner::getNavPoseVec]\n" <<
      "\t updateRotationAngle failed!\n";
    return false;
  }

  if(pose_vec.size() == 0)
  {
    return true;
  }

  nav_pose_vec.emplace_back(pose_vec[0]);

  if(pose_vec.size() == 1)
  {
    return true;
  }

  geometry_msgs::Pose new_pose = nav_pose_vec.back();

  for(size_t i = 1; i < pose_vec.size(); ++i)
  {
    const geometry_msgs::Pose& target_pose = pose_vec[i];
    bool need_face_to_target_pose = false;
    if(i == pose_vec.size() - 1)
    {
      need_face_to_target_pose = true;
    }

    std::vector<geometry_msgs::Pose> single_nav_pose_vec;
    if(!getSingleNavPoseVec(
          new_pose, target_pose, need_face_to_target_pose, single_nav_pose_vec))
    {
      std::cout << "[ERROR][RobotPathPlanner::getNavPoseVec]\n" <<
        "\t getSingleNavPoseVec failed!\n";
      return false;
    }

    if(single_nav_pose_vec.size() > 0)
    {
      for(const geometry_msgs::Pose& pose : single_nav_pose_vec)
      {
        nav_pose_vec.emplace_back(pose);
      }
    }

    new_pose = nav_pose_vec.back();
  }

  return true;
}

bool RobotPathPlanner::updateParam(
    const double& delta_rotation_angle,
    const double& delta_move_dist)
{
  if(delta_rotation_angle <= 0)
  {
    std::cout << "[ERROR][RobotPathPlanner::updateRotationAngle]\n" <<
      "\t delta_rotation_angle <= 0!\n";
    return false;
  }
  if(delta_move_dist <= 0)
  {
    std::cout << "[ERROR][RobotPathPlanner::updateRotationAngle]\n" <<
      "\t delta_move_dist <= 0!\n";
    return false;
  }

  delta_rotation_angle_ = delta_rotation_angle;
  delta_move_dist_ = delta_move_dist;

  delta_rotation_quat_ = tf2::Quaternion(
      tf2::Vector3(0, 0, 1), delta_rotation_angle_);
  delta_opposite_rotation_quat_ = tf2::Quaternion(
      tf2::Vector3(0, 0, 1), -delta_rotation_angle_);

  return true;
}

double RobotPathPlanner::getPointNorm2(
    const geometry_msgs::Point& point)
{
  const double point_norm2 =
    point.x * point.x +
    point.y * point.y +
    point.z * point.z;
  return point_norm2;
}

double RobotPathPlanner::getPointNorm(
    const geometry_msgs::Point& point)
{
  const double point_norm2 = getPointNorm2(point);
  return std::sqrt(point_norm2);
}

bool RobotPathPlanner::getUnitPoint(
    const geometry_msgs::Point& source_point,
    geometry_msgs::Point& unit_point)
{
  const double norm_min = 1e-3;

  const double point_norm = getPointNorm(source_point);
  if(point_norm < norm_min)
  {
    // std::cout << "[WARN][RobotPathPlanner::getUnitPoint]\n" <<
    //   "\t point_norm < " << norm_min << "!\n";
    return false;
  }

  unit_point.x = source_point.x / point_norm;
  unit_point.y = source_point.y / point_norm;
  unit_point.z = source_point.z / point_norm;
  return true;
}

bool RobotPathPlanner::getUnitAngle(
    const double& source_angle,
    double& unit_angle)
{
  unit_angle = source_angle;

  while (unit_angle > M_PI)
  {
    unit_angle -= 2.0 * M_PI;
  }
  while (unit_angle < -M_PI)
  {
    unit_angle += 2.0 * M_PI;
  }
  return true;
}

bool RobotPathPlanner::getPoseFaceToAngle(
    const geometry_msgs::Pose& pose,
    double& face_to_angle)
{
  tf::Quaternion quat_pose;
  tf::quaternionMsgToTF(pose.orientation, quat_pose);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat_pose).getRPY(roll, pitch, yaw);
  face_to_angle = yaw;
  return true;
}

bool RobotPathPlanner::getPoseFaceToDirection(
    const geometry_msgs::Pose& pose,
    geometry_msgs::Point& face_to_direction)
{
  double face_to_angle;
  if(!getPoseFaceToAngle(pose, face_to_angle))
  {
    std::cout << "[ERROR][RobotPathPlanner::getPoseFaceToDirection]\n" <<
      "\t getPoseFaceToAngle failed!\n";
    return false;
  }

  face_to_direction.x = cos(face_to_angle);
  face_to_direction.y = sin(face_to_angle);
  face_to_direction.z = 0;
  return true;
}

bool RobotPathPlanner::getRotatePoseVec(
    const geometry_msgs::Pose& source_pose,
    const geometry_msgs::Point& target_direction,
    std::vector<geometry_msgs::Pose>& new_pose_vec)
{
  new_pose_vec.clear();

  geometry_msgs::Point unit_direction;
  if(!getUnitPoint(target_direction, unit_direction))
  {
    // std::cout << "[WARN][RobotPathPlanner::getRotatePoseVec]\n" <<
    //   "\t getUnitPoint failed!\n";
    return true;
  }

  const double source_pose_face_to_angle =
    atan2(unit_direction.y, unit_direction.x);

  double yaw;
  if(!getPoseFaceToAngle(source_pose, yaw))
  {
    std::cout << "[ERROR][RobotPathPlanner::getSingleNavPoseVec]\n" <<
      "\t getPoseFaceToAngle failed!\n";
    return false;
  }

  const double rotate_angle = source_pose_face_to_angle - yaw;
  double unit_rotate_angle;
  if(!getUnitAngle(rotate_angle, unit_rotate_angle))
  {
    std::cout << "[ERROR][RobotPathPlanner::getSingleNavPoseVec]\n" <<
      "\t getUnitAngle failed!\n";
    return false;
  }

  if(unit_rotate_angle == 0)
  {
    return true;
  }

  geometry_msgs::Pose new_pose = source_pose;

  const size_t rotate_num = std::floor(
      std::fabs(unit_rotate_angle) / delta_rotation_angle_);

  tf2::Quaternion quat_middle_state;
  tf2::fromMsg(new_pose.orientation, quat_middle_state);

  tf2::Quaternion quat_delta_rotation;
  double last_rotate_angle;

  if (unit_rotate_angle >= 0)
  {
    last_rotate_angle = unit_rotate_angle -
      rotate_num * delta_rotation_angle_;
    quat_delta_rotation = delta_rotation_quat_;
  }
  else
  {
    last_rotate_angle = unit_rotate_angle +
      rotate_num * delta_rotation_angle_;
    quat_delta_rotation = delta_opposite_rotation_quat_;
  }

  if (rotate_num > 0)
  {
    for (size_t i = 0; i < rotate_num; ++i)
    {
      quat_middle_state *= quat_delta_rotation;
      new_pose.orientation = tf2::toMsg(quat_middle_state);
      new_pose_vec.emplace_back(new_pose);
    }
  }

  if (last_rotate_angle != 0)
  {
    quat_middle_state *= tf2::Quaternion(tf2::Vector3(0, 0, 1), last_rotate_angle);
    new_pose.orientation = tf2::toMsg(quat_middle_state);
    new_pose_vec.emplace_back(new_pose);
  }

  return true;
}

bool RobotPathPlanner::getMovePoseVec(
    const geometry_msgs::Pose& source_pose,
    const geometry_msgs::Point& target_position,
    std::vector<geometry_msgs::Pose>& new_pose_vec)
{
  new_pose_vec.clear();

  geometry_msgs::Point move_direction;
  move_direction.x = target_position.x - source_pose.position.x;
  move_direction.y = target_position.y - source_pose.position.y;
  move_direction.z = target_position.z - source_pose.position.z;
  const double move_dist = getPointNorm(move_direction);

  if(move_dist <= 0)
  {
    return true;
  }

  geometry_msgs::Point unit_move_direction;
  if(!getUnitPoint(move_direction, unit_move_direction))
  {
    // std::cout << "[WARN][RobotPathPlanner::getMovePoseVec]\n" <<
    //   "\t getUnitPoint failed!\n";
    return true;
  }

  geometry_msgs::Pose new_pose = source_pose;

  const size_t move_num = std::floor(1.0 * move_dist / delta_move_dist_);

  const double last_move_dist = move_dist - move_num * delta_move_dist_;

  if (move_num > 0)
  {
    for (size_t j = 1; j <= move_num; ++j)
    {
      new_pose.position.x += delta_move_dist_ * unit_move_direction.x;
      new_pose.position.y += delta_move_dist_ * unit_move_direction.y;
      new_pose.position.z += delta_move_dist_ * unit_move_direction.z;

      new_pose_vec.emplace_back(new_pose);
    }
  }

  if (last_move_dist > 0)
  {
    new_pose.position.x += last_move_dist * unit_move_direction.x;
    new_pose.position.y += last_move_dist * unit_move_direction.y;
    new_pose.position.z += last_move_dist * unit_move_direction.z;

    new_pose_vec.emplace_back(new_pose);
  }

  return true;
}

bool RobotPathPlanner::getSingleNavPoseVec(
    const geometry_msgs::Pose& source_pose,
    const geometry_msgs::Pose& target_pose,
    const bool& need_face_to_target_pose,
    std::vector<geometry_msgs::Pose>& nav_pose_vec)
{
  nav_pose_vec.clear();

  const double step_position_x_diff = target_pose.position.x - source_pose.position.x;
  const double step_position_y_diff = target_pose.position.y - source_pose.position.y;

  geometry_msgs::Pose new_pose = source_pose;

  geometry_msgs::Point face_to_direction;
  face_to_direction.x = step_position_x_diff;
  face_to_direction.y = step_position_y_diff;
  face_to_direction.z = 0;

  std::vector<geometry_msgs::Pose> new_pose_vec;
  if(!getRotatePoseVec(new_pose, face_to_direction, new_pose_vec))
  {
    std::cout << "[ERROR][RobotPathPlanner::getSingleNavPoseVec]\n" <<
      "\t getRotatePoseVec for first direction failed!\n";
    return false;
  }
  if(new_pose_vec.size() > 0)
  {
    for(const geometry_msgs::Pose& pose : new_pose_vec)
    {
      nav_pose_vec.emplace_back(pose);
    }
    new_pose = new_pose_vec.back();
  }

  if(!getMovePoseVec(new_pose, target_pose.position, new_pose_vec))
  {
    std::cout << "[ERROR][RobotPathPlanner::getSingleNavPoseVec]\n" <<
      "\t getMovePoseVec failed!\n";
    return false;
  }
  if(new_pose_vec.size() > 0)
  {
    for(const geometry_msgs::Pose& pose : new_pose_vec)
    {
      nav_pose_vec.emplace_back(pose);
    }
    new_pose = new_pose_vec.back();
  }

  if (!need_face_to_target_pose)
  {
    return true;
  }

  if(!getPoseFaceToDirection(target_pose, face_to_direction))
  {
    std::cout << "[ERROR][RobotPathPlanner::getSingleNavPoseVec]\n" <<
      "\t getPoseFaceToDirection failed!\n";
    return false;
  }
  if(!getRotatePoseVec(new_pose, face_to_direction, new_pose_vec))
  {
    std::cout << "[ERROR][RobotPathPlanner::getSingleNavPoseVec]\n" <<
      "\t getRotatePoseVec for second direction failed!\n";
    return false;
  }
  if(new_pose_vec.size() > 0)
  {
    for(const geometry_msgs::Pose& pose : new_pose_vec)
    {
      nav_pose_vec.emplace_back(pose);
    }
  }

  return true;
}

