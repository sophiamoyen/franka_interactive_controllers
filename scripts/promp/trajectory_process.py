import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import os
import time
import tf
from mpl_toolkits.mplot3d import Axes3D

class TrajectoryProcess:
    '''
        Pipelines:
            1. Kinesthetic teaching -> bag -> bag2traj_save -> load_traj -> plot_traj (Inspect recorded trajectory)
            2. load_traj -> promp_training -> condition -> plot_traj
    '''
    def __init__(self):
        self.traj = None
        self.traj_type = None
        self.cwd = os.getcwd()

    def _combine_time(self, secs, nsecs):
        '''
            Combine the secs and nsecs to a float number
        '''
        return secs + (nsecs * 10e-10)

    def _bag2traj(self, bag_path, topic_type):
        '''
            Read the bag file and return the trajectory according to topic 
        '''
        bag = rosbag.Bag(bag_path)
        msg_nums = bag.get_type_and_topic_info()[1][topic_type][1]
        if topic_type == '/franka_state_controller/joint_states':
            '''
                Trajectories for training promp
            '''
            joint_traj_array = np.zeros((msg_nums, 15)) # [time, y1, ... y7, yd1, ... yd7]
            for i, (topic, msg, t) in enumerate(bag.read_messages(topics=[topic_type])):
                com_time = self._combine_time(msg.header.stamp.secs, msg.header.stamp.nsecs)
                joint_traj_array[i, 0] = com_time
                joint_traj_array[i, 1:8] = msg.position
                joint_traj_array[i, 8:15] = msg.velocity

            traj_type = 'joint_states'
            return joint_traj_array, traj_type
        
        elif topic_type == '/franka_state_controller/O_T_EE':
            '''
                Only for plotting purpose
            '''
            cart_traj_array = np.zeros((msg_nums, 7)) # [time, x, y, z, roll, pitch, yaw]
            for i, (topic, msg, t) in enumerate(bag.read_messages(topics=[topic_type])):
                com_time = self._combine_time(msg.header.stamp.secs, msg.header.stamp.nsecs)
                cart_traj_array[i, 0] = com_time
                quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quat)
                cartesian_pose = np.array([msg.pose.position.x,
                                           msg.pose.position.y, 
                                           msg.pose.position.z, 
                                           euler[0], 
                                           euler[1], 
                                           euler[2]])
                for dof in range(0, len(cartesian_pose)):
                    cart_traj_array[i, dof+1] = cartesian_pose[dof]

            traj_type = 'O_T_EE'
            return cart_traj_array, traj_type
        bag.close()

    def bag2traj_save(self, i, bag_path, topic_type, save_path):
        '''
            Save the trajectory as a .npz file from bag file
        '''
        traj, traj_type = self._bag2traj(bag_path, topic_type)
        if traj_type == 'joint_states':
            np.savez(save_path, joint_states=traj)
        elif traj_type == 'O_T_EE':
            np.savez(save_path, O_T_EE=traj)

        print(f"Trajectory saved in {save_path}")

    def load_traj(self, traj_path:list):
        # TODO: load multiple trajectories
        '''
            Load the trajectory from a .npz file
        '''
        traj_data = np.load(traj_path)
        traj_type = traj_data.files
        traj = np.array(traj_data[traj_type[0]])
        print(f"Trajectory loaded from {traj_path}, shape: {traj.shape}")
        self.traj = traj
        self.traj_type = traj_type[0]
        

    def plot_traj(self, sys_type='linux', **kwargs):
        '''
            Plot the trajectory
        '''
        fig_size = kwargs['fig_size'] if 'fig_size' in kwargs else (10, 10)
        sampled_traj = kwargs['sampled_traj'] if 'sampled_traj' in kwargs else None
        sampled_traj_type = kwargs['sampled_traj_type'] if 'sampled_traj_type' in kwargs else None

        if self.traj is not None:
            '''
                Plot the trajectory from the loaded .npz file
            '''
            traj = self.traj
            traj_type = self.traj_type

        elif sampled_traj is not None:
            '''
                Plot the trajectory from the sampled trajectory
            '''
            assert self.traj is None
            traj = sampled_traj
            traj_type = sampled_traj_type

        if traj_type == 'O_T_EE':
            fig = plt.figure(figsize=fig_size)
            ori_every = kwargs['ori_every'] if 'ori_every' in kwargs else 100
            ori_length = kwargs['ori_length'] if 'ori_length' in kwargs else 0.015
            conditioned_points = kwargs['conditioned_points'] if 'conditioned_points' in kwargs else None
            azim = kwargs['azim'] if 'azim' in kwargs else 45
            elev = kwargs['elev'] if 'elev' in kwargs else 30
            legend = kwargs['legend'] if 'legend' in kwargs else False

            ax = Axes3D(fig) if sys_type == 'linux' else fig.add_subplot(projection='3d')
            ax.plot(traj[:, 1], traj[:, 2], traj[:, 3], label='Cartesian trajectory')
            ax.quiver(traj[::ori_every, 1], 
                      traj[::ori_every, 2], 
                      traj[::ori_every, 3], 
                      traj[::ori_every, 4], 
                      traj[::ori_every, 5], 
                      traj[::ori_every, 6], 
                      length=ori_length,)
            ax.scatter(traj[0, 1], traj[0, 2], traj[0, 3], label='Start point', color='g', s=100)
            ax.scatter(traj[-1, 1], traj[-1, 2], traj[-1, 3], label='End point', color='r', s=100)
            for i in len(conditioned_points):
                ax.scatter(conditioned_points[i][0], conditioned_points[i][1], conditioned_points[i][2], label='Conditioned point', color='blue', s=100)
            
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.view_init(azim=azim, elev=elev)

            if legend:
                ax.legend()
        elif traj_type == 'joint_states':
            # TODO: finish this
            fig, axs = plt.subplots(1, 7, figsize=fig_size)
            # increase the space between subplots
            fig.subplots_adjust(hspace=0.5)
            for dof in range(7):
                for i in range(n_samples):
                    axs[dof].plot(sample_time[i], promp_samples[i][:,dof], color='green', label='ProMP Unconditioned')
                    axs[dof].plot(sample_time[i], cond_samples[i][:,dof], color='blue', label=f'ProMP Conditioned')
                    axs[dof].plot(sample_time[i], task_cond_samples[i][:,dof], color='orange', label=f'ProMP cartesian conditioned')
                    axs[dof].plot(0.7, q_cond_init[dof], 's', color='red', markersize=10, alpha=0.5)
                    axs[dof].plot(0, q_cond_init[dof], 's', color='red', markersize=10, alpha=0.5)
                    # axs[dof].legend()
                    # set titel for subplots
                    axs[dof].set_title(f'Joint {dof}')


# TODO: implement training pipeline of promp



        
