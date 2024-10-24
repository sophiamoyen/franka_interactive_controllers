// This code was derived from franka_example controllers
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license.
// Current development and modification of this code by Nadia Figueroa (MIT) 2021.

#include <cartesian_pose_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <pseudo_inversion.h>
#include <hardware_interface/joint_command_interface.h>

namespace franka_interactive_controllers {

bool CartesianPoseImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_desired_pose_ = node_handle.subscribe(
      "/cartesian_impedance_controller/desired_pose", 20, &CartesianPoseImpedanceController::desiredPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_external_wrench_ = node_handle.subscribe(
      "/franka_state_controller/F_ext", 20, &CartesianPoseImpedanceController::externalWrenchCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  this->error_pub_.init(node_handle, "/cartesian_impedance_controller/cartesian_error", 1);
  this->error_unclipped_pub_.init(node_handle, "/cartesian_impedance_controller/cartesian_error_unclipped", 1);

  // Getting ROSParams
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianPoseImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianPoseImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // Getting libranka control interfaces
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianPoseImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Getting Dynamic Reconfigure objects
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      // dynamic_reconfigure::Server<franka_interactive_controllers::minimal_compliance_paramConfig>>(
      dynamic_reconfigure::Server<franka_interactive_controllers::compliance_full_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianPoseImpedanceController::complianceParamCallback, this, _1, _2));


  // Initializing variables
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  ///////////////////////////////////////////////////////////////////////////
  ////////////////  Parameter Initialization from YAML FILES!!!     /////////
  ///////////////////////////////////////////////////////////////////////////

   // Initialize stiffness and damping gains
  cartesian_stiffness_target_.setIdentity();
  cartesian_damping_target_.setIdentity();
  std::vector<double> cartesian_stiffness_target_yaml;
  if (!node_handle.getParam("cartesian_stiffness_target", cartesian_stiffness_target_yaml) || cartesian_stiffness_target_yaml.size() != 6) {
    ROS_ERROR(
      "CartesianPoseImpedanceController: Invalid or no cartesian_stiffness_target_yaml parameters provided, "
      "aborting controller init!");
    return false;
  }
  for (int i = 0; i < 6; i ++) {
    cartesian_stiffness_target_(i,i) = cartesian_stiffness_target_yaml[i];
  }
  // Damping ratio = 1
  default_cart_stiffness_target_ << 500, 500, 500, 50, 50, 50;
  for (int i = 0; i < 6; i ++) {
    if (cartesian_stiffness_target_yaml[i] == 0.0)
      cartesian_damping_target_(i,i) = 2.0 * sqrt(default_cart_stiffness_target_[i]);
    else
      cartesian_damping_target_(i,i) = 2.0 * sqrt(cartesian_stiffness_target_yaml[i]);
  }
  ROS_INFO_STREAM("cartesian_stiffness_target_: " << std::endl <<  cartesian_stiffness_target_);
  ROS_INFO_STREAM("cartesian_damping_target_: " << std::endl <<  cartesian_damping_target_);

  if (!node_handle.getParam("nullspace_stiffness", nullspace_stiffness_target_) || nullspace_stiffness_target_ <= 0) {
    ROS_ERROR(
      "CartesianPoseImpedanceController: Invalid or no nullspace_stiffness parameters provided, "
      "aborting controller init!");
    return false;
  }
  ROS_INFO_STREAM("nullspace_stiffness_target_: " << std::endl <<  nullspace_stiffness_target_);

  // Initialize variables for tool compensation from yaml config file
  activate_tool_compensation_ = true;
  tool_compensation_force_.setZero();
  std::vector<double> external_tool_compensation;
  // tool_compensation_force_ << 0.46, -0.17, -1.64, 0, 0, 0;  //read from yaml
  if (!node_handle.getParam("external_tool_compensation", external_tool_compensation) || external_tool_compensation.size() != 6) {
      ROS_ERROR(
          "CartesianPoseImpedanceController: Invalid or no external_tool_compensation parameters provided, "
          "aborting controller init!");
      return false;
    }
  for (size_t i = 0; i < 6; ++i) 
    tool_compensation_force_[i] = external_tool_compensation.at(i);
  ROS_INFO_STREAM("External tool compensation force: " << std::endl << tool_compensation_force_);

  // Initialize variables for nullspace control from yaml config file
  q_d_nullspace_.setZero();
  std::vector<double> q_nullspace;
  if (node_handle.getParam("q_nullspace", q_nullspace)) {
    q_d_nullspace_initialized_ = true;
    if (q_nullspace.size() != 7) {
      ROS_ERROR(
        "CartesianPoseImpedanceController: Invalid or no q_nullspace parameters provided, "
        "aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 7; ++i) 
      q_d_nullspace_[i] = q_nullspace.at(i);
    ROS_INFO_STREAM("Desired nullspace position (from YAML): " << std::endl << q_d_nullspace_);
  }
  return true;
}

void CartesianPoseImpedanceController::starting(const ros::Time& /*time*/) {

  // Get robot current/initial joint state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set desired point to current state
  position_d_           = initial_transform.translation();
  orientation_d_        = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_    = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  if (!q_d_nullspace_initialized_) {
    q_d_nullspace_ = q_initial;
    q_d_nullspace_initialized_ = true;
    ROS_INFO_STREAM("Desired nullspace position (from q_initial): " << std::endl << q_d_nullspace_);
  }
}

void CartesianPoseImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_tool(6), tau_ext(7);

  tau_ext = jacobian.transpose() * external_wrench_;
  // smoothing tau_ext
  double smooth_factor_tau_ext = 0.01;
  tau_ext = smooth_factor_tau_ext * tau_ext + (1.0 - smooth_factor_tau_ext) * tau_ext_old_;
  

  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> error_orientation_rectified;
  error.head(3) << position - position_d_;


  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);

  // error clipping & smoothing
  double pos_clip_factor = 0.05;
  double ori_clip_factor = 0.2;
  for (int i = 0; i < 3; i++) {
    if (error[i] > pos_clip_factor) {
      error[i] = pos_clip_factor;
    } else if (error[i] < -pos_clip_factor) {
      error[i] = -pos_clip_factor;
    }
  }
  for (int i = 3; i < 6; i++) {
    if (error[i] > ori_clip_factor) {
      error[i] = ori_clip_factor;
    } else if (error[i] < -ori_clip_factor) {
      error[i] = -ori_clip_factor;
    }
  }

  double smooth_factor = 0.5;
  error = smooth_factor * error + (1.0 - smooth_factor) * error_old_;


  if (this->error_pub_.trylock()) {
    geometry_msgs::PoseStamped error_msg;
    error_msg.header.stamp = ros::Time::now();
    error_msg.pose.position.x = error[0];
    error_msg.pose.position.y = error[1];
    error_msg.pose.position.z = error[2];
    error_msg.pose.orientation.x = error[3];
    error_msg.pose.orientation.y = error[4];
    error_msg.pose.orientation.z = error[5];
    error_msg.pose.orientation.w = 0.0;
    this->error_pub_.msg_ = error_msg;
    this->error_pub_.unlockAndPublish();
  }


  // Cartesian PD control with damping ratio = 1
  Eigen::Matrix<double, 6, 1> velocity;
  velocity << jacobian * dq;
  Eigen::VectorXd     F_ee_des_;
  F_ee_des_.resize(6);
  error_orientation_rectified = error;
  // put an additional factor as rotation and translation live on a different scale - therefore we amplify the error
  error_orientation_rectified.tail(3) << 1.5*error_orientation_rectified.tail(3);
  F_ee_des_ << -cartesian_stiffness_ * error_orientation_rectified - cartesian_damping_ * velocity;
  tau_task << jacobian.transpose() * F_ee_des_;
  
  // kinematic pseudoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // nullspace PD control with damping ratio = 1
  // ROS_WARN_STREAM_THROTTLE(0.5, "Nullspace stiffness:" << nullspace_stiffness_); 
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);

  // Compute tool compensation (scoop/camera in scooping task)
  if (activate_tool_compensation_)
    tau_tool << jacobian.transpose() * tool_compensation_force_;
  else
    tau_tool.setZero();

//  // Desired torque sent to the motors
//  ROS_INFO_STREAM_THROTTLE(0.5, "Tau_task: " << tau_task.norm());
//  ROS_INFO_STREAM_THROTTLE(0.5, "Tau_nullspace: " << tau_nullspace.norm());
//  ROS_INFO_STREAM_THROTTLE(0.5, "Tau_tool: " << tau_tool.norm());
//  ROS_INFO_STREAM_THROTTLE(0.5, "Coriolis: " << coriolis.norm());
  
  // tau_d << tau_task + tau_nullspace + coriolis + tau_ext;
  tau_d << tau_task + tau_nullspace + coriolis;
//  ROS_WARN_STREAM_THROTTLE(0.5, "torque from external wrench + control: \n" << tau_d+tau_ext);
//  ROS_INFO_STREAM_THROTTLE(0.5, "Desired control torque:" << tau_d);
//  ROS_INFO_STREAM_THROTTLE(0.5, "REFERENCE:" << position);
  // ROS_WARN_STREAM_THROTTLE(0.5, "Desired control torque:" << tau_d.transpose());

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
  
  error_old_ = error;
  tau_ext_old_ = tau_ext;
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  // ROS_INFO_STREAM("Desired position: \n" << position_d_target_ << std::endl << "Cartesian stiffness: \n" << cartesian_stiffness_);
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = 2*filter_params_ * position_d_target_ + (1.0 - 2*filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(2*filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CartesianPoseImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianPoseImpedanceController::complianceParamCallback(
    // franka_interactive_controllers::minimal_compliance_paramConfig& config,
    franka_interactive_controllers::compliance_full_paramConfig& config,
    uint32_t /*level*/) {
  nullspace_stiffness_target_ = config.nullspace_stiffness;
  activate_tool_compensation_ = config.activate_tool_compensation;

  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_(0, 0) = config.x_TRANSLATIONAL_stiffness;
  cartesian_stiffness_target_(1, 1) = config.y_TRANSLATIONAL_stiffness;
  cartesian_stiffness_target_(2, 2) = config.z_TRANSLATIONAL_stiffness;
  cartesian_stiffness_target_(3, 3) = config.x_ROTATIONAL_stiffness;
  cartesian_stiffness_target_(4, 4) = config.y_ROTATIONAL_stiffness;
  cartesian_stiffness_target_(5, 5) = config.z_ROTATIONAL_stiffness;
  cartesian_damping_target_.setIdentity();
  cartesian_damping_target_(0, 0) = 2.0 * sqrt(config.x_TRANSLATIONAL_stiffness);
  cartesian_damping_target_(1, 1) = 2.0 * sqrt(config.y_TRANSLATIONAL_stiffness);
  cartesian_damping_target_(2, 2) = 2.0 * sqrt(config.z_TRANSLATIONAL_stiffness);
  cartesian_damping_target_(3, 3) = 2.0 * sqrt(config.x_ROTATIONAL_stiffness);
  cartesian_damping_target_(4, 4) = 2.0 * sqrt(config.y_ROTATIONAL_stiffness);
  cartesian_damping_target_(5, 5) = 2.0 * sqrt(config.z_ROTATIONAL_stiffness);

  // cartesian_stiffness_target_.setIdentity();
  // cartesian_stiffness_target_.topLeftCorner(3, 3)
  //     << config.all_TRANSLATIONAL_stiffness * Eigen::Matrix3d::Identity();
  // cartesian_stiffness_target_.bottomRightCorner(3, 3)
  //     << config.all_ROTATIONAL_stiffness * Eigen::Matrix3d::Identity();
  // cartesian_damping_target_.setIdentity();
  // // Damping ratio = 1
  // cartesian_damping_target_.topLeftCorner(3, 3)
  //     << 2.0 * sqrt(config.all_TRANSLATIONAL_stiffness) * Eigen::Matrix3d::Identity();
  // cartesian_damping_target_.bottomRightCorner(3, 3)
  //     << 2.0 * sqrt(config.all_ROTATIONAL_stiffness) * Eigen::Matrix3d::Identity();
}

void CartesianPoseImpedanceController::externalWrenchCallback(
    const geometry_msgs::WrenchStampedConstPtr& msg) {
  double contact_x = msg->wrench.force.x;
  if (contact_x > 4.0) {
    ROS_WARN_STREAM_THROTTLE(0.5, "Contact detected: " << contact_x);
    contact_x = 0.5 * contact_x;
  }
  external_wrench_ << contact_x, msg->wrench.force.y, msg->wrench.force.z,
                      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  // external_wrench_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
  //                     msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
}

void CartesianPoseImpedanceController::desiredPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  traj_timestamp = msg->header.stamp;
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  // ROS_INFO_STREAM("[CALLBACK] Desired ee position from DS: " << position_d_target_);
  
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

// void CartesianPoseImpedanceController::desiredPoseCallback(
//     const geometry_msgs::PoseArrayConstPtr& msg) {
//   std::lock_guard<std::mutex> position_d_target_mutex_lock(
//       position_and_orientation_d_target_mutex_);
//   traj_timestamp = msg->header.stamp;

//   bool pose_inc = true;
//   int pose_id;
//   if (pose_inc) 
//   {
//     pose_id = 1;
//   }
//   else
//   {
//     pose_id = 0;
//   }
//   position_d_target_ << msg->poses[pose_id].position.x, msg->poses[pose_id].position.y, msg->poses[pose_id].position.z;
//   Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
//   orientation_d_target_.coeffs() << msg->poses[pose_id].orientation.x, msg->poses[pose_id].orientation.y,
//       msg->poses[pose_id].orientation.z, msg->poses[pose_id].orientation.w;

//   // x_increment
//   // position_d_inc_ << msg->poses[1].position.x, msg->poses[1].position.y, msg->poses[1].position.z;
//   // orientation_d_inc_.coeffs() << msg->poses[1].orientation.x, msg->poses[1].orientation.y,
//   //     msg->poses[1].orientation.z, msg->poses[1].orientation.w;

//   if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
//     orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
//   }
// }

}  // namespace franka_interactive_controllers

PLUGINLIB_EXPORT_CLASS(franka_interactive_controllers::CartesianPoseImpedanceController,
                       controller_interface::ControllerBase)
