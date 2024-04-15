#!/usr/bin/env python

import sys
import rospy as ros

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ReloadControllerLibraries
from geometry_msgs.msg import PoseStamped


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
    ros.init_node('move_to_start')

    load_controller("effort_joint_trajectory_controller")
    switch_controller(["effort_joint_trajectory_controller"], ["cartesian_pose_impedance_controller"])

    action = ros.resolve_name('effort_joint_trajectory_controller/follow_joint_trajectory')
    client = SimpleActionClient(action, FollowJointTrajectoryAction)
    ros.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
    client.wait_for_server()

    param = ros.resolve_name('~joint_pose')
    pose = ros.get_param(param, None)
    if pose is None:
        ros.logerr('move_to_start: Could not find required parameter "' + param + '"')
        sys.exit(1)

    topic = ros.resolve_name('franka_state_controller/joint_states')
    ros.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
    joint_state = ros.wait_for_message(topic, JointState)
    initial_pose = dict(zip(joint_state.name, joint_state.position))

    max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

    point = JointTrajectoryPoint()
    point.time_from_start = ros.Duration.from_sec(
        # Use either the time to move the furthest joint with 'max_dq' or 500ms,
        # whatever is greater
        max(max_movement / 0.3, 0.5)
    )
    goal = FollowJointTrajectoryGoal()

    goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
    point.velocities = [0] * len(pose)

    goal.trajectory.points.append(point)
    goal.goal_time_tolerance = ros.Duration.from_sec(0.5)

    ros.loginfo('Sending trajectory Goal to move into initial config')
    client.send_goal_and_wait(goal)

    result = client.get_result()
    if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
        ros.logerr('move_to_start: Movement was not successful: ' + {
            FollowJointTrajectoryResult.INVALID_GOAL:
            """
            The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
            Is the 'joint_pose' reachable?
            """,

            FollowJointTrajectoryResult.INVALID_JOINTS:
            """
            The joint pose you specified is for different joints than the joint trajectory controller
            is claiming. Does you 'joint_pose' include all 7 joints of the robot?
            """,

            FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
            """
            During the motion the robot deviated from the planned path too much. Is something blocking
            the robot?
            """,

            FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
            """
            After the motion the robot deviated from the desired goal pose too much. Probably the robot
            didn't reach the joint_pose properly
            """,
        }[result.error_code])

    else:
        ros.loginfo('move_to_start: Successfully moved into start pose')
        switch_controller(["cartesian_pose_impedance_controller"], ["effort_joint_trajectory_controller"])
        unload_controller("effort_joint_trajectory_controller")
