#!/usr/bin/env python3

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

        # added some variables to prevent consequtive calling - TODO: maybe improve using mutex!
        self.gripper_open = None
        self.currently_opening = False
        self.currently_closing = False

        # first initialize everything
        self.startup_procedure()

        self.sub_gripper = rospy.Subscriber('/cartesian_impedance_controller/desired_gripper_state', PointStamped, self.gripper_callback, queue_size=1)

        # then run online
        rospy.spin()

    def gripper_callback(self, data):
        # data = data.data
        gripper_open = not(data.point.x==1)
        # if it is switching at the moment - ignore
        if not(self.currently_closing) and not(self.currently_closing):
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
        if not(self.currently_closing) and not(self.currently_opening):
            self.currently_opening = True
            node_process = Popen(shlex.split('rosrun franka_interactive_controllers franka_gripper_run_node 1'))
            # messagebox.showinfo("Open Gripper", "Gripper Opened")
            time.sleep(1)  # Sleep for 1 seconds
            node_process.terminate()
            time.sleep(1)
            self.gripper_open = True
            self.currently_opening = False

    def close_gripper(self):
        if not (self.currently_closing) and not (self.currently_opening):
            self.currently_closing = True
            node_process = Popen(shlex.split('rosrun franka_interactive_controllers franka_gripper_run_node 0'))
            # messagebox.showinfo("Close Gripper", "Gripper Closed")
            time.sleep(1)  # Sleep for 1 seconds
            node_process.terminate()
            time.sleep(1)
            self.gripper_open = False
            self.currently_closing = False

if __name__ == '__main__':
    try:
        runobj = GripperControl()
    except rospy.ROSInterruptException:
        pass

