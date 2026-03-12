// Refered to https://github.com/frankaemika/franka_ros/tree/develop/franka_example_controllers

#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

namespace serl_franka_controllers {

class JointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  void commandCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};
  std::array<double, 7> reset_pose_{};

  // Streaming joint position commands — second-order (spring-damper) filter
  ros::Subscriber command_sub_;
  std::array<double, 7> target_positions_{};
  std::array<double, 7> commanded_positions_{};
  std::array<double, 7> commanded_velocities_{};
  bool streaming_mode_{false};

  // Spring-damper parameters
  double spring_k_{100.0};
  double damping_d_{20.0};
  double max_acceleration_{8.0};  // rad/s^2
  double max_velocity_{0.5};      // rad/s — caps peak speed for large movements
};

}  // namespace serl_franka_controllers
