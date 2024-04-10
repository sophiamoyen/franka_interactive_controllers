// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>
#include <Eigen/Dense>

#include <franka_interactive_controllers/minimal_compliance_paramConfig.h>
#include <franka_interactive_controllers/compliance_full_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_interactive_controllers {

class CartesianPoseImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 1> default_cart_stiffness_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 6, 1> error_old_ = Eigen::Matrix<double, 6, 1>::Zero();
  ros::Time traj_timestamp{0.0};

  std::mutex position_and_orientation_d_target_mutex_;

  // whether to load from yaml or use initial robot config
  bool q_d_nullspace_initialized_ = false;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  // Variables for initialization and tool compensation
  Eigen::Matrix<double, 6, 1> tool_compensation_force_;
  bool activate_tool_compensation_;
  
  // Dynamic reconfigure
//   std::unique_ptr<dynamic_reconfigure::Server<franka_interactive_controllers::minimal_compliance_paramConfig>>
  std::unique_ptr<dynamic_reconfigure::Server<franka_interactive_controllers::compliance_full_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_interactive_controllers::compliance_full_paramConfig& config,
//   void complianceParamCallback(franka_interactive_controllers::minimal_compliance_paramConfig& config,
                               uint32_t level);

  // Desireds pose subscriber
  ros::Subscriber sub_desired_pose_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> error_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> error_unclipped_pub_;

  void desiredPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  // void desiredPoseCallback(const geometry_msgs::PoseArrayConstPtr& msg);
};

}  // namespace franka_interactive_controllers
