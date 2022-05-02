#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin, sqrt
import numpy as np
from time import time, sleep

import rospy
from tf import transformations
from gazebo_msgs.srv import GetModelState

from tensorboard_logger_ros.msg import Scalar
from tensorboard_logger_ros.srv import ScalarToBool

class RobotMoveStateManager(object):
    def __init__(self):
        self.robot_name = None
        self.robot_num = None

        self.robot_move_dist_list = None

        sleep(10)
        self.get_model_state_proxy = \
            rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.tf_logger_proxy = \
            rospy.ServiceProxy('/tensorboard_logger/log_scalar', ScalarToBool)
        return

    def loadRobot(self, robot_name, robot_num):
        self.robot_name = robot_name
        self.robot_num = robot_num

        self.robot_move_dist_list = []
        for _ in range(self.robot_num):
            self.robot_move_dist_list.append(0)
        return True

    def logScalar(self, name, step, value):
        scalar = Scalar()
        scalar.name = str(name)
        scalar.step = int(step)
        scalar.value = float(value)
        log_success = self.tf_logger_proxy(scalar)
        return log_success

    def getRobotState(self, robot_name):
        robot_state = self.get_model_state_proxy(robot_name, "")
        return robot_state

    def getEulerAngleFromQuaternion(self, quaternion):
        (roll, pitch, yaw) = transformations.euler_from_quaternion([
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]])

        return np.array([roll, pitch, yaw])

    def getQuaternionFromEulerAngle(self, euler_angle):
        quaternion = transformations.quaternion_from_euler(
            euler_angle[0], euler_angle[1], euler_angle[2])
        return np.array([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])

    def getRotationMatrixFromEulerAngle(self, euler_angle):
        R_x = np.array([
            [1, 0, 0],
            [0, cos(euler_angle[0]), -sin(euler_angle[0])],
            [0, sin(euler_angle[0]), cos(euler_angle[0])]
        ])
                    
        R_y = np.array([
            [cos(euler_angle[1]), 0, sin(euler_angle[1])],
            [0, 1, 0],
            [-sin(euler_angle[1]), 0, cos(euler_angle[1])]
        ])
                    
        R_z = np.array([
            [cos(euler_angle[2]), -sin(euler_angle[2]), 0],
            [sin(euler_angle[2]), cos(euler_angle[2]), 0],
            [0, 0, 1]
        ])
                    
        rotation_matrix = np.dot(R_z, np.dot( R_y, R_x ))
        return rotation_matrix

    def getForwardDirection(self, robot_state):
        x_axis_direction = np.array([1, 0, 0])

        robot_orientation = robot_state.pose.orientation

        robot_quaternion = [
            robot_orientation.x,
            robot_orientation.y,
            robot_orientation.z,
            robot_orientation.w]

        euler_angle = self.getEulerAngleFromQuaternion(robot_quaternion)

        rotation_matrix = self.getRotationMatrixFromEulerAngle(euler_angle)

        forward_direction = np.dot(rotation_matrix, x_axis_direction)
        forward_direction = np.array([forward_direction[0], forward_direction[1], 0])
        forward_direction_norm = np.linalg.norm(forward_direction)

        if forward_direction_norm == 0:
            print("[ERROR][RobotMoveStateManager::getForwardDirection]")
            print("\t forward_direction_norm is 0!")
            return None

        forward_direction /= forward_direction_norm
        return forward_direction

    def getAllRobotState(self):
        robot_state_list = []

        if self.robot_name is None:
            print("[ERROR][RobotMoveStateManager::getAllRobotState]")
            print("\t robot_name is None!")
            return None

        if self.robot_num is None:
            print("[ERROR][RobotMoveStateManager::getAllRobotState]")
            print("\t robot_num is None!")
            return None

        if self.robot_num < 1:
            print("[ERROR][RobotMoveStateManager::getAllRobotState]")
            print("\t robot_num not valid!")
            return None

        for robot_idx in range(self.robot_num):
            current_robot_full_name = self.robot_name + str(robot_idx)
            current_robot_state = self.getRobotState(current_robot_full_name)
            if current_robot_state is None:
                print("[ERROR][RobotMoveStateManager::getAllRobotState]")
                print("\t getRobotState for " + current_robot_full_name + " failed!")
                return None
            robot_state_list.append(current_robot_state)
        return robot_state_list

    def getPosisionDiff2(self, state_1, state_2):
        position_1 = state_1.pose.position
        position_2 = state_2.pose.position

        position_x_diff = position_1.x - position_2.x
        position_y_diff = position_1.y - position_2.y
        position_z_diff = position_1.z - position_2.z

        position_diff2 = \
            position_x_diff * position_x_diff + \
            position_y_diff * position_y_diff + \
            position_z_diff * position_z_diff
        return position_diff2

    def getOrientationDiff2(self, state_1, state_2):
        orientation_1 = state_1.pose.orientation
        orientation_2 = state_2.pose.orientation

        orientation_x_diff = orientation_1.x - orientation_2.x
        orientation_y_diff = orientation_1.y - orientation_2.y
        orientation_z_diff = orientation_1.z - orientation_2.z
        orientation_w_diff = orientation_1.w - orientation_2.w

        orientation_diff2 = \
            orientation_x_diff * orientation_x_diff + \
            orientation_y_diff * orientation_y_diff + \
            orientation_z_diff * orientation_z_diff + \
            orientation_w_diff * orientation_w_diff
        return orientation_diff2

    def getPosisionDiff(self, state_1, state_2):
        position_diff2 = self.getPosisionDiff2(state_1, state_2)
        return sqrt(position_diff2)

    def getOrientationDiff(self, state_1, state_2):
        orientation_diff2 = self.getOrientationDiff2(state_1, state_2)
        return sqrt(orientation_diff2)

    def isSameState(self, state_1, state_2):
        position_diff2_max = 0.0001
        orientation_diff2_max = 0.0001

        position_diff2 = self.getPosisionDiff2(state_1, state_2)
        if position_diff2 > position_diff2_max:
            return False

        orientation_diff2 = self.getOrientationDiff2(state_1, state_2)
        if orientation_diff2 > orientation_diff2_max:
            return False

        return True

    def startListenRobotState(self):
        robot_wait_count_min_time = 10

        robot_wait_time_sum = 0
        log_start_time = time()
        last_log_time = 0

        last_robot_state_list = []
        robot_wait_count_list = []
        for _ in range(self.robot_num):
            robot_wait_count_list.append(0)

        while True:
            last_start_time = time()
            sleep(0.1)

            new_robot_state_list = self.getAllRobotState()
            if new_robot_state_list is None:
                if len(last_robot_state_list) == 0:
                    continue
                print("[ERROR][RobotMoveStateManager::startListenRobotState]")
                print("\t getAllRobotState failed!")
                break

            if len(new_robot_state_list) != self.robot_num:
                print("[ERROR][RobotMoveStateManager::startListenRobotState]")
                print("\t new_robot_state_list.size and robot_num not matched!")
                break

            if len(last_robot_state_list) == 0:
                last_robot_state_list = new_robot_state_list
                continue

            for i in range(self.robot_num):
                if self.isSameState(
                    last_robot_state_list[i],
                    new_robot_state_list[i]):
                    robot_wait_count_list[i] += 1
                else:
                    robot_wait_count_list[i] = 0

                self.robot_move_dist_list[i] += self.getPosisionDiff(
                    last_robot_state_list[i],
                    new_robot_state_list[i])

            last_robot_state_list = new_robot_state_list

            exist_robot_wait = False
            for robot_wait_count in robot_wait_count_list:
                if robot_wait_count < robot_wait_count_min_time:
                    continue
                exist_robot_wait = True
                break

            if exist_robot_wait:
                new_wait_time = time() - last_start_time
                robot_wait_time_sum += new_wait_time

            new_log_time = time()
            if new_log_time == last_log_time:
                continue

            last_log_time = new_log_time
            if not self.logScalar(
                "RobotMoveStateManager/robot_wait_time",
                new_log_time - log_start_time,
                robot_wait_time_sum):
                print("[ERROR][RobotMoveStateManager::startListenRobotState]")
                print("\t logScalar for robot_wait_time failed!")
                break

            for i in range(self.robot_num):
                if not self.logScalar(
                    "RobotMoveStateManager/robot" + str(i) + "_move_dist",
                    new_log_time - log_start_time,
                    self.robot_move_dist_list[i]):
                    print("[ERROR][RobotMoveStateManager::startListenRobotState]")
                    print("\t logScalar for robot_" + str(i) + "_move_dist failed!")
                    break

            robot_move_dist_array = np.array(self.robot_move_dist_list)
            robot_move_dist_mean = np.mean(robot_move_dist_array)
            robot_move_dist_std = np.std(robot_move_dist_array)
            if robot_move_dist_mean == 0:
                continue

            robot_cov = robot_move_dist_std / robot_move_dist_mean
            if not self.logScalar(
                "RobotMoveStateManager/robot_wait_time",
                new_log_time - log_start_time,
                robot_cov):
                print("[ERROR][RobotMoveStateManager::startListenRobotState]")
                print("\t logScalar for robot_cov failed!")
                break
        return True

if __name__ == "__main__":
    rospy.init_node("RobotMoveStateManager")
    robot_name = rospy.get_param("/robot_name")
    robot_num = int(rospy.get_param("/robot_num"))

    robot_move_state_manager = RobotMoveStateManager()
    robot_move_state_manager.loadRobot(robot_name, robot_num)
    robot_move_state_manager.startListenRobotState()

