#!/usr/bin/env python

import sys
import rospy as ros
import numpy as np

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ReloadControllerLibraries
from geometry_msgs.msg import PoseStamped
from util import go_to


def load_controller(controller_name):
    ros.wait_for_service("/controller_manager/load_controller")
    load_controller = ros.ServiceProxy("/controller_manager/load_controller", LoadController)
    load_controller(controller_name)

def unload_controller(controller_name):
    ros.wait_for_service("/controller_manager/unload_controller")
    unload_controller = ros.ServiceProxy("/controller_manager/unload_controller", UnloadController)
    unload_controller(controller_name)

def switch_controller(start_controllers, stop_controllers):
    ros.wait_for_service("/controller_manager/switch_controller")
    switch_controller = ros.ServiceProxy("/controller_manager/switch_controller", SwitchController)
    switch_controller(start_controllers, stop_controllers, 0, False, 0.0)

if __name__ == '__main__':
    ros.init_node('move_to_start', anonymous=True)

    load_controller("effort_joint_trajectory_controller")
    switch_controller(["effort_joint_trajectory_controller"], ["cartesian_pose_impedance_controller"])

    action = ros.resolve_name('effort_joint_trajectory_controller/follow_joint_trajectory')
    client = SimpleActionClient(action, FollowJointTrajectoryAction)
    ros.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
    client.wait_for_server()

    topic = ros.resolve_name('franka_state_controller/joint_states')
    ros.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
    joint_state = ros.wait_for_message(topic, JointState)
    initial_pose = dict(zip(joint_state.name, joint_state.position))

    result = go_to(client, joint_state.name[:7], joint_state.position[:7],
         np.array([0.004286136549292948, 0.23023615878924988, -0.003981800034836296, -1.7545947008261213, 0.0032928755527341326, 1.994446315732633, 0.7839058620188021]), duration=5)
    print (result)
    if (result.error_code == FollowJointTrajectoryResult.SUCCESSFUL):
        print ("The robot was successfully moved and initialized")
    else:
        # roserror message
        ros.logerr("The robot was not able to move and initialized")

    load_controller("cartesian_pose_impedance_controller")
    switch_controller(["cartesian_pose_impedance_controller"], ["effort_joint_trajectory_controller"])
    # unload_controller("effort_joint_trajectory_controller")

    # just make sure to wait until the rest is over
    ros.spin()

