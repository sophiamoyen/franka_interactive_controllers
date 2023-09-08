//
// Created by changqi on 9/6/23.
//

#ifndef SRC_DMP_IMPEDANCE_CONTROLLER_H
#define SRC_DMP_IMPEDANCE_CONTROLLER_H

#endif //SRC_DMP_IMPEDANCE_CONTROLLER_H

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_interactive_controllers/minimal_compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_interactive_controllers {
    class DmpImpedanceController : public controller_interface::MultiInterfaceController<
                                          franka_hw::FrankaModelInterface,
                                          hardware_interface::EffortJointInterface,
                                          franka_hw::FrankaStateHandle> {
      public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;

      private:
        // Saturation
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(
                const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                const Eigen::Matrix<double, 7, 1>& tau_J_d);

        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelInterface> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;


    };


}