#!/usr/bin/env python

import sys
import rospy
import numpy as np
import tf
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import copy
import time

from std_srvs.srv import Trigger, TriggerResponse

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ReloadControllerLibraries
from geometry_msgs.msg import PoseStamped
from util import go_to

class TeleopInterface:
    def __init__(self):
        rospy.init_node('teleop_interface', anonymous=True)
        self.transform_listener = tf.TransformListener()
        time.sleep(1)
        self.pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=0)
        self.turned_on = False
        self.initialized = False
        self.hand_homogeneous = np.eye(4)
        self.ee_homogeneous = np.eye(4)
        self.rate = rospy.Rate(50)

        self.toggle_status = rospy.Service('/toggle_teleop_mode', Trigger, self.toggle_status_fct)

        self.run_online()

    def toggle_status_fct(self, request):
        self.turned_on = not self.turned_on
        # if it is deactivated - also remove that it is initialized
        if not(self.turned_on):
            self.initialized = False
        return TriggerResponse(success=True, message="Toggled Teleop Status - teleop status now: " + str(self.turned_on))

    def run_online(self):
        while not rospy.is_shutdown():
            if self.turned_on:
                try:
                    ee_pos, ee_quat = self.transform_listener.lookupTransform('panda_link0', 'panda_NE', rospy.Time(0.))
                    hand_pos, hand_quat = self.transform_listener.lookupTransform('panda_link0', 'RightHand', rospy.Time(0.))
                    self.ee_homogeneous[:3,:3] = quaternion_matrix(ee_quat)[:-1, :-1]
                    self.ee_homogeneous[:3,3] = ee_pos
                    self.hand_homogeneous[:3,:3] = quaternion_matrix(hand_quat)[:-1, :-1]
                    self.hand_homogeneous[:3,3] = hand_pos

                    if not(self.initialized):
                        self.initialized = True
                        self.ref_hand_pos = copy.deepcopy(hand_pos)
                        self.ref_hand_quat = copy.deepcopy(hand_quat)
                        self.ref_hand_homogeneous = np.eye(4)
                        self.ref_hand_homogeneous[:3, 3] = self.ref_hand_pos
                        self.ref_hand_homogeneous[:3, :3] = quaternion_matrix(self.ref_hand_quat)[:-1, :-1]
                        print (self.ref_hand_homogeneous)
                        self.ref_hand_homogeneous_inv = np.eye(4)
                        self.ref_hand_homogeneous_inv[:3,:3] = np.linalg.inv(self.ref_hand_homogeneous[:3,:3])
                        self.ref_hand_homogeneous_inv[:3, 3] = -np.matmul(np.linalg.inv(self.ref_hand_homogeneous[:3,:3]),self.ref_hand_homogeneous[:3, 3])
                        self.ref_ee_pos = copy.deepcopy(ee_pos)
                        self.ref_ee_quat = copy.deepcopy(ee_quat)
                        self.ref_ee_homogeneous = np.eye(4)
                        self.ref_ee_homogeneous[:3, 3] = self.ref_ee_pos
                        self.ref_ee_homogeneous[:3,:3] = quaternion_matrix(self.ref_ee_quat)[:-1, :-1]

                    if (self.initialized):
                        # create geometry_msgs/PoseStamped
                        msg = PoseStamped()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = 'panda_link0'
                        # add some scaling such that the robot can be moved more easily
                        scale_factor_rot = 1.75
                        msg.pose.position.x = self.ref_ee_pos[0] + scale_factor_rot*(hand_pos[0] - self.ref_hand_pos[0])
                        msg.pose.position.y = self.ref_ee_pos[1] + scale_factor_rot*(hand_pos[1] - self.ref_hand_pos[1])
                        msg.pose.position.z = self.ref_ee_pos[2] + scale_factor_rot*(hand_pos[2] - self.ref_hand_pos[2])
                        # msg.pose.orientation.x = self.ref_ee_quat[0]
                        # msg.pose.orientation.y = self.ref_ee_quat[1]
                        # msg.pose.orientation.z = self.ref_ee_quat[2]
                        # msg.pose.orientation.w = self.ref_ee_quat[3]

                        action_matrix = np.matmul(self.hand_homogeneous, self.ref_hand_homogeneous_inv)
                        des_grip_pose = np.matmul(action_matrix, self.ref_ee_homogeneous)

                        quat = quaternion_from_matrix(des_grip_pose)

                        # msg.pose.position.x = des_grip_pose[0,3]
                        # msg.pose.position.y = des_grip_pose[1,3]
                        # msg.pose.position.z = des_grip_pose[2,3]
                        msg.pose.orientation.x = quat[0]
                        msg.pose.orientation.y = quat[1]
                        msg.pose.orientation.z = quat[2]
                        msg.pose.orientation.w = quat[3]


                        self.pub.publish(msg)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    # log
                    rospy.logerr('Could not receive the transform')
                    continue

                self.rate.sleep()
            else:
                # more generous sleep if it is not on
                time.sleep(1)







if __name__ == '__main__':
    runobj = TeleopInterface()

