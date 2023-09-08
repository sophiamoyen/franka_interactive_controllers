//
// Created by changqi on 9/6/23.
//

#include "dmp_impedance_controller.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <pseudo_inversion.h>
#include <hardware_interface/joint_command_interface.h>






PLUGINLIB_EXPORT_CLASS(franka_interactive_controllers::CartesianPoseImpedanceController,
                       controller_interface::ControllerBase)