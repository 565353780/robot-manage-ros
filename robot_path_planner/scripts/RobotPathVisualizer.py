#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
#  import pcl
#  import pcl.pcl_visualization as viewer
from time import sleep
from random import randint
import numpy as np

import rospy
from robot_path_planner.msg import PoseVec

COLOR_MAP = np.array([
    [0., 0., 0.],
    [174., 199., 232.],
    [152., 223., 138.],
    [31., 119., 180.],
    [255., 187., 120.],
    [188., 189., 34.],
    [140., 86., 75.],
    [255., 152., 150.],
    [214., 39., 40.],
    [197., 176., 213.],
    [148., 103., 189.],
    [196., 156., 148.],
    [23., 190., 207.],
    [100., 85., 144.],
    [247., 182., 210.],
    [66., 188., 102.],
    [219., 219., 141.],
    [140., 57., 197.],
    [202., 185., 52.],
    [51., 176., 203.],
    [200., 54., 131.],
    [92., 193., 61.],
    [78., 71., 183.],
    [172., 114., 82.],
    [255., 127., 14.],
    [91., 163., 138.],
    [153., 98., 156.],
    [140., 153., 101.],
    [158., 218., 229.],
    [100., 125., 154.],
    [178., 127., 135.],
    [146., 111., 194.],
    [44., 160., 44.],
    [112., 128., 144.],
    [96., 207., 209.],
    [227., 119., 194.],
    [213., 92., 176.],
    [94., 106., 211.],
    [82., 84., 163.],
    [100., 85., 144.]
])

class RobotPathVisualizer(object):
    def __init__(self):
        self.save_pose_vec_num = 4

        self.nav_pose_vec_list = []

        self.nav_pose_vec_sub = \
            rospy.Subscriber(
                '/robot_path_planner/nav_pose_vec',
                PoseVec,
                self.getNavPoseVecCallback)
        return

    def getNavPoseVecCallback(self, nav_pose_vec):
        self.nav_pose_vec_list.append(nav_pose_vec.pose_vec)
        if len(self.nav_pose_vec_list) > self.save_pose_vec_num:
            self.nav_pose_vec_list.pop()
        return True

    def getNavPositionArrayList(self):
        nav_position_array_list = []
        for nav_pose_vec in self.nav_pose_vec_list:
            nav_position_list = []
            for nav_pose in nav_pose_vec:
                position = nav_pose.position
                nav_position_list.append([
                    position.x,
                    position.y,
                    position.z])
            nav_position_array = np.array(nav_position_list)
            nav_position_array_list.append(nav_position_array)
        return nav_position_array_list

    def showPoseVecWithCV(self):
        image_x_min = -10
        image_x_max = 10
        image_y_min = -10
        image_y_max = 10
        zoom = 50

        nav_position_array_list = self.getNavPositionArrayList()
        image = np.zeros(
            (zoom * (image_x_max - image_x_min),
             zoom * (image_y_max - image_y_min), 3),
            dtype=np.uint8)
        for nav_position_array in nav_position_array_list:
            if nav_position_array.shape[0] == 0:
                continue
            random_color_idx = randint(0, 39)
            nav_position_array_in_image = nav_position_array[:, :2] * zoom
            nav_position_array_in_image[:, 0] -= zoom * image_x_min
            nav_position_array_in_image[:, 1] -= zoom * image_y_min
            cv2.polylines(image,
                          np.int32([nav_position_array_in_image]),
                          False,
                          COLOR_MAP[random_color_idx],
                          2)

        cv2.imshow("RobotPathVisualizer", image)
        cv2.waitKey(1)
        return True

    #  def showPoseVecWithPCL(self):
    #      vs=pcl.pcl_visualization.PCLVisualizering
    #      vss=pcl.pcl_visualization.PCLVisualizering()
    #
    #      nav_position_array_list = self.getNavPositionArrayList()
    #
    #      cloud_idx = 0
    #      for nav_position_array in nav_position_array_list:
    #          random_color_idx = randint(0, COLOR_MAP.shape[0] - 1)
    #          cloud = pcl.PointCloud(nav_position_array)
    #          color = COLOR_MAP[random_color_idx]
    #          visualcolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(
    #              cloud, color[0], color[1], color[2])
    #          vs.AddPointCloud_ColorHandler(
    #              vss, cloud, visualcolor, id='cloud' + str(cloud_idx), viewport=0)
    #          cloud_idx += 1
    #      vss.SpinOnce()
    #      for i in range(len(nav_position_array_list)):
    #          vss.RemovePointCloud('cloud' + str(i), 0)
    #      return True

    def startShowPoseVec(self):
        while True:
            if len(self.nav_pose_vec_list) == 0:
                sleep(1)
                continue
            break

        while True:
            sleep(0.5)
            self.showPoseVecWithCV()
            #  self.showPoseVecWithPCL()

if __name__ == "__main__":
    rospy.init_node("RobotPathVisualizer")

    robot_path_visualizer = RobotPathVisualizer()
    robot_path_visualizer.startShowPoseVec()

