#!/usr/bin/env python
# -*- coding: utf-8 -*-

import open3d as o3d

import rospy
from robot_path_planner.msg import PoseVec

class RobotPathVisualizer(object):
    def __init__(self):
        self.nav_pose_vec_list = []

        self.nav_pose_vec_sub = \
            rospy.Subscriber(
                '/robot_path_planner/nav_pose_vec',
                PoseVec,
                self.getNavPoseVecCallback)
        return

    def getNavPoseVecCallback(self, nav_pose_vec):
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print(nav_pose_vec.pose_vec)
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        self.nav_pose_vec_list.append(nav_pose_vec)
        return True

if __name__ == "__main__":
    rospy.init_node("RobotPathVisualizer")

    robot_path_visualizer = RobotPathVisualizer()

    rospy.spin()

