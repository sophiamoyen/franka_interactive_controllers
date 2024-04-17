import copy
import rospy
import actionlib
import numpy as np
from threading import Lock
from typing import List, Union
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pinocchio as pin
from typing import Optional


def go_to(trajectory_client: actionlib.SimpleActionClient, joint_names: List[str],
          init_positions: Union[np.ndarray, List[float]], target_positions: Union[np.ndarray, List[float]],
          duration: float = None, max_vel: float = None, wait: bool = True, time_offset: float = 0.0):
    if duration is None:
        if max_vel is None:
            raise RuntimeError("If duration is not specified, the maximum allowed joint velocity must be specified")

        duration = max(0.5, np.linalg.norm(np.array(init_positions) - np.array(target_positions)) / max_vel)

    # Create a trajectory in which we simply move J0 by 20 degrees
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(time_offset)
    traj.joint_names = joint_names

    traj_point = JointTrajectoryPoint()
    traj_point.time_from_start = rospy.Duration(duration)
    traj_point.positions = target_positions
    traj_point.velocities = [0] * len(target_positions)
    traj.points = [traj_point]

    traj_goal = FollowJointTrajectoryGoal()
    traj_goal.trajectory = traj
    if wait:
        trajectory_client.send_goal_and_wait(traj_goal)
        return trajectory_client.get_result()
    else:
        trajectory_client.send_goal(traj_goal)


class JointStateListener:

    def __init__(self):
        self.sub = rospy.Subscriber("/joint_states", JointState, self._ros_callback)
        self.last_message = None
        self.lock = Lock()

    def _ros_callback(self, data: JointState):
        self.lock.acquire()
        self.last_message = data
        self.lock.release()

    def get_data(self) -> JointState:
        self.lock.acquire()
        data = copy.deepcopy(self.last_message)
        self.lock.release()
        return data

class InverseKinematicsHelper:

    def __init__(self, urdf_string: str, frame_name: str):
        self.model = pin.buildModelFromXML(urdf_string)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(frame_name)

    def get_joint_position(self, target_pos: np.ndarray, target_rot: np.ndarray, q_init: Optional[np.ndarray] = None):
        oMdes = pin.SE3(target_rot, target_pos)
        q = pin.neutral(self.model)
        if q_init is not None:
            q[:q_init.shape[0]] = q_init

        eps = 1e-4
        IT_MAX = 1000
        DT = 1e-1
        damp = 1e-12

        i = 0
        while True:
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            dMi = oMdes.actInv(self.data.oMf[self.frame_id])
            err = pin.log(dMi).vector
            if np.linalg.norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break
            J = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id)
            v = - J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(self.model, q, v * DT)
            if not i % 50:
                print('%d: error = %s' % (i, err.T))
            i += 1

        if not success:
            raise RuntimeError("Inverse Kinematics not successful!")

        return q.flatten()
