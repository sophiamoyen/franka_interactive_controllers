#!/usr/bin/env python
import rosbag
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_matrix
import os
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np
import tf



def play_bag_cartesian_impedance_control():
    node_name = "play_bags" 
    rospy.init_node(node_name, anonymous=True)
    bag_root = "/home/changi/Code/tactile_bags/single_target_match_igniting/target0"
    bag_name = "2024-03-22-14-38-36.bag"
    bag_path = f"{bag_root}/{bag_name}"
    bag = rosbag.Bag(bag_path)
    cartesian_pub = rospy.Publisher("/cartesian_impedance_controller/desired_pose", PoseStamped, queue_size=1)
    # read bag
    # get pub rate
    ee_pose_recording_rate = bag.get_type_and_topic_info()[1]["/franka_state_controller/ee_pose"][1] / (bag.get_end_time() - bag.get_start_time())
    pub_rate = rospy.Rate(ee_pose_recording_rate)
    for topic, msg, t in bag.read_messages(topics=["/franka_state_controller/ee_pose"]):
        msg_ee_pose = PoseStamped()
        msg_ee_pose.header.stamp = rospy.Time.now()
        msg_ee_pose.header.frame_id = "panda_link0"
        msg_ee_pose.pose.position.x = msg.position.x
        msg_ee_pose.pose.position.y = msg.position.y
        msg_ee_pose.pose.position.z = msg.position.z
        msg_ee_pose.pose.orientation.x = msg.orientation.x
        msg_ee_pose.pose.orientation.y = msg.orientation.y
        msg_ee_pose.pose.orientation.z = msg.orientation.z
        msg_ee_pose.pose.orientation.w = msg.orientation.w
        print(msg_ee_pose)
        cartesian_pub.publish(msg_ee_pose)
        pub_rate.sleep()
        # if dected ctrl c break
        if rospy.is_shutdown():
            bag.close()


    bag.close()

def play_bag_cartesian_pose_control():
    node_name = "play_bags" 
    rospy.init_node(node_name, anonymous=True)
    bag_root = "/home/changi/Code/tactile_bags/single_target_match_igniting/target0"
    bag_name = "2024-03-22-14-27-59.bag"
    bag_path = f"{bag_root}/{bag_name}"
    bag = rosbag.Bag(bag_path)
    cartesian_pub = rospy.Publisher("/cartesian_pose_controller/desired_pose", Float64MultiArray, queue_size=1)
    # read bag
    # get pub rate
    ee_pose_recording_rate = bag.get_type_and_topic_info()[1]["/franka_state_controller/ee_pose"][1] / (bag.get_end_time() - bag.get_start_time())
    pub_rate = rospy.Rate(ee_pose_recording_rate)
    for topic, msg, t in bag.read_messages(topics=["/franka_state_controller/ee_pose"]):
        msg_ee_vec = Float64MultiArray()
        ee_pos = [msg.position.x, msg.position.y, msg.position.z]
        ee_ori = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        ee_mat = tf.transformations.compose_matrix(translate=ee_pos, angles=tf.transformations.euler_from_quaternion(ee_ori))
        ee_vec = ee_mat.T.flatten()
        print("ee_mat\n", ee_mat)
        print("ee_vec\n", ee_vec)
        msg_ee_vec.data = ee_vec
        msg_ee_vec.layout.data_offset = 0
        # msg_ee_vec.layout.dim.label = "ee_vec"
        # msg_ee_vec.layout.dim.size = 1
        # msg_ee_vec.layout.dim.stride = 0
            

        cartesian_pub.publish(msg_ee_vec)
        pub_rate.sleep()
        # if dected ctrl c break
        if rospy.is_shutdown():
            bag.close()

    bag.close()

if __name__ == "__main__":
    play_bag_cartesian_impedance_control()
    rospy.spin()

