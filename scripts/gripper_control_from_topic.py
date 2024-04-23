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

class GripperControl:
    def __init__(self):
        rospy.init_node('teleop_interface', anonymous=True)

        # retrieve name of the node
        self.name = rospy.get_name()
        self.namespace = rospy.get_namespace()

        # potentially retrieve information from yaml file -> not done in current state
        # self.device_path = rospy.get_param(rospy.get_name() + "/device_path")

        # first initialize everything
        self.startup_procedure()


        self.gripper_open = None
        self.sub_gripper = rospy.Subscriber('/cartesian_impedance_controller/desired_gripper_state', PointStamped, self.gripper_callback)

        # then run online
        rospy.spin()

    def gripper_callback(self, data):
        # data = data.data
        gripper_open = not(data.point.x==1)
        if self.gripper_open:
            if not(gripper_open):
                self.close_gripper()
        elif not(self.gripper_open):
            if (gripper_open):
                self.open_gripper()

    def startup_procedure(self):
        # open gripper first
        self.open_gripper()

    def open_gripper(self):
        node_process = Popen(shlex.split('rosrun franka_interactive_controllers franka_gripper_run_node 1'))
        # messagebox.showinfo("Open Gripper", "Gripper Opened")
        time.sleep(1)  # Sleep for 1 seconds
        node_process.terminate()
        self.gripper_open = True

    def close_gripper(self):
        node_process = Popen(shlex.split('rosrun franka_interactive_controllers franka_gripper_run_node 0'))
        # messagebox.showinfo("Close Gripper", "Gripper Closed")
        time.sleep(1)  # Sleep for 1 seconds
        node_process.terminate()
        self.gripper_open = False

if __name__ == '__main__':
    try:
        runobj = GripperControl()
    except rospy.ROSInterruptException:
        pass

