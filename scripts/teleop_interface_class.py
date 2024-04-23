#!/usr/bin/env python

import sys
import rospy
import numpy as np
import tf
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import copy
import time
from psutil import Popen
import shlex

from std_srvs.srv import Trigger, TriggerResponse

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ReloadControllerLibraries
from geometry_msgs.msg import PoseStamped, PointStamped
from util import go_to

class TeleopInterface:
    def __init__(self):
        rospy.init_node('teleop_interface', anonymous=True)

        # retrieve name of the node
        self.name = rospy.get_name()
        self.namespace = rospy.get_namespace()

        # potentially retrieve information from yaml file -> not done in current state
        # self.device_path = rospy.get_param(rospy.get_name() + "/device_path")

        self.transform_listener = tf.TransformListener()
        time.sleep(1)
        self.pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=0)
        self.pub_current_pose = rospy.Publisher('/cartesian_impedance_controller/current_pose', PoseStamped, queue_size=0)
        self.pub_gripper = rospy.Publisher('/cartesian_impedance_controller/desired_gripper_state', PointStamped, queue_size=0)
        self.teleop_turned_on = False
        self.initialized = False
        self.gripper_open = None
        self.first_time = True
        self.hand_homogeneous = np.eye(4)
        self.ee_homogeneous = np.eye(4)
        self.rate = rospy.Rate(50)

        self.toggle_status = rospy.Service('/toggle_teleop_mode', Trigger, self.toggle_teleop_status)

        # first initialize everything
        self.startup_procedure()

        self.toggle_gripper_state = rospy.Service('/toggle_gripper_state', Trigger, self.toggle_gripper_status)
        self.move_to_home_srv = rospy.Service('/move_to_home', Trigger, self.move_to_home)

        # then run online
        self.run_online()

    def startup_procedure(self):
        action = rospy.resolve_name('effort_joint_trajectory_controller/follow_joint_trajectory')
        client = SimpleActionClient(action, FollowJointTrajectoryAction)
        rospy.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
        client.wait_for_server()

        topic = rospy.resolve_name('franka_state_controller/joint_states')
        rospy.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
        joint_state = rospy.wait_for_message(topic, JointState)
        initial_pose = dict(zip(joint_state.name, joint_state.position))

        # open gripper first
        self.open_gripper_w_msg()

        # then go to default pose
        result = go_to(client, joint_state.name[:7], joint_state.position[:7],
                       np.array([0.004286136549292948, 0.23023615878924988, -0.003981800034836296, -1.7545947008261213,
                                 0.0032928755527341326, 1.994446315732633, 0.7839058620188021]), duration=5)
        print(result)
        if (result.error_code == FollowJointTrajectoryResult.SUCCESSFUL):
            print("The robot was successfully moved and initialized")
        else:
            # roserror message
            rospy.logerr("The robot was not able to move and initialized")


        # after moving to the desired pose, switch the controller to the effort joint trajectory!
        if (self.first_time):
            self.load_controller("cartesian_pose_impedance_controller")
            self.first_time = False
        self.switch_controller(["cartesian_pose_impedance_controller"], ["effort_joint_trajectory_controller"])


    def load_controller(self, controller_name):
        rospy.wait_for_service("/controller_manager/load_controller")
        load_controller = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
        load_controller(controller_name)

    def unload_controller(self, controller_name):
        rospy.wait_for_service("/controller_manager/unload_controller")
        unload_controller = rospy.ServiceProxy("/controller_manager/unload_controller", UnloadController)
        unload_controller(controller_name)

    def switch_controller(self, start_controllers, stop_controllers):
        rospy.wait_for_service("/controller_manager/switch_controller")
        switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        switch_controller(start_controllers, stop_controllers, 0, False, 0.0)

    def open_gripper(self):
        self.gripper_open = True

    def open_gripper_w_msg(self):
        # function is needed as otherwise the desired gripper state is only sent upon the node being active!
        msg_gripper = PointStamped()
        msg_gripper.header.stamp = rospy.Time.now()
        self.gripper_open = True
        msg_gripper.point.x = float(not (self.gripper_open))
        self.pub_gripper.publish(msg_gripper)
        time.sleep(1)


    def close_gripper(self):
        self.gripper_open = False


    def toggle_teleop_status(self, request):
        self.teleop_turned_on = not self.teleop_turned_on
        # if it is deactivated - also remove that it is initialized
        if not(self.teleop_turned_on):
            self.initialized = False
        return TriggerResponse(success=True, message="Toggled Teleop Status - teleop status now: " + str(self.teleop_turned_on))

    def toggle_gripper_status(self, request):
        if self.teleop_turned_on:
            if (self.gripper_open):
                self.close_gripper()
            else:
                self.open_gripper()

            return TriggerResponse(success=True, message="Toggled Gripper State - gripper state now: " + str(self.gripper_open))

    def move_to_home(self, request):
        # also, make sure to disable the teleop stuff!
        self.teleop_turned_on = False
        self.initialized = False

        # first we have to change the gripper status back
        self.switch_controller(["effort_joint_trajectory_controller"], ["cartesian_pose_impedance_controller"])
        # then we can execute the normal go home move
        self.startup_procedure()

        return TriggerResponse(success=True, message="Panda was moved to its home / default configuration")

    def run_online(self):
        while not rospy.is_shutdown():
            if self.teleop_turned_on:
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
                        curr_time = rospy.Time.now()
                        msg.header.stamp = curr_time
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

                        msg_curr_pose = PoseStamped()
                        msg_curr_pose.header.stamp = curr_time
                        msg_curr_pose.header.frame_id = 'panda_link0'
                        msg_curr_pose.pose.position.x = ee_pos[0]
                        msg_curr_pose.pose.position.y = ee_pos[1]
                        msg_curr_pose.pose.position.z = ee_pos[2]
                        msg_curr_pose.pose.orientation.x = ee_quat[0]
                        msg_curr_pose.pose.orientation.y = ee_quat[1]
                        msg_curr_pose.pose.orientation.z = ee_quat[2]
                        msg_curr_pose.pose.orientation.w = ee_quat[3]
                        self.pub_current_pose.publish(msg_curr_pose)

                        msg_gripper = PointStamped()
                        msg_gripper.header.stamp = curr_time
                        msg_gripper.point.x = float(not(self.gripper_open))
                        self.pub_gripper.publish(msg_gripper)

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

