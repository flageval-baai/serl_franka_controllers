/*
Refered to Source file:
  https://github.com/frankaemika/franka_ros/blob/develop/franka_example_controllers/src/joint_position_example_controller.cpp
*/

#include <serl_franka_controllers/joint_position_controller.h>

#include <algorithm>
#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace serl_franka_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::vector<double> target_positions;
  if (!node_handle.getParam("/target_joint_positions", target_positions) || target_positions.size() != 7) {
    ROS_ERROR("JointPositionController: Could not read target joint positions from parameter server or incorrect size");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    reset_pose_[i] = target_positions[i];
  }

  // Second-order filter: virtual spring-damper (same principle as impedance controller)
  // K=100, D=20 gives critical damping with ~0.2s rise time, good for 10Hz streaming
  // max_acceleration/max_velocity clamp for large steps to stay within Franka limits
  node_handle.param<double>("joint_spring_k", spring_k_, 100.0);
  node_handle.param<double>("joint_damping_d", damping_d_, 20.0);
  node_handle.param<double>("joint_max_acceleration", max_acceleration_, 8.0);
  node_handle.param<double>("joint_max_velocity", max_velocity_, 0.5);

  ROS_INFO("JointPositionController: spring_k=%.1f, damping_d=%.1f, max_accel=%.1f, max_vel=%.1f",
           spring_k_, damping_d_, max_acceleration_, max_velocity_);

  // Subscribe to streaming joint commands
  command_sub_ = node_handle.subscribe(
      "/joint_position_controller/command", 1,
      &JointPositionController::commandCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

// Same pattern as CartesianImpedanceController::equilibriumPoseCallback
void JointPositionController::commandCallback(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if (msg->data.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: command size " << msg->data.size() << ", expected 7");
    return;
  }
  for (size_t i = 0; i < 7; ++i) {
    target_positions_[i] = msg->data[i];
  }
  streaming_mode_ = true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
    commanded_positions_[i] = initial_pose_[i];
    commanded_velocities_[i] = 0.0;
    target_positions_[i] = initial_pose_[i];
  }
  elapsed_time_ = ros::Duration(0.0);
  streaming_mode_ = false;
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  if (streaming_mode_) {
    // Second-order (spring-damper) filter:
    //   accel = K * (target - pos) - D * vel
    //
    // This produces smooth position, velocity, AND acceleration.
    // Same physical principle as the CartesianImpedanceController
    // but applied as a trajectory filter for position commands.
    //
    // For small VLA deltas (~0.003 rad at 30Hz):
    //   accel = 100 * 0.003 = 0.3 rad/s^2 -> natural, smooth
    // For large steps (0.75 rad):
    //   accel = 100 * 0.75 = 75 -> clamped to 10 rad/s^2 -> safe ramp
    double dt = period.toSec();

    for (size_t i = 0; i < 7; ++i) {
      double error = target_positions_[i] - commanded_positions_[i];

      // Spring-damper acceleration
      double accel = spring_k_ * error - damping_d_ * commanded_velocities_[i];

      // Safety clamp for large steps
      accel = std::clamp(accel, -max_acceleration_, max_acceleration_);

      // Integrate velocity, then clamp peak speed
      commanded_velocities_[i] += accel * dt;
      commanded_velocities_[i] = std::clamp(commanded_velocities_[i],
                                            -max_velocity_, max_velocity_);

      // Integrate position (always smooth since velocity is continuous)
      commanded_positions_[i] += commanded_velocities_[i] * dt;

      position_joint_handles_[i].setCommand(commanded_positions_[i]);
    }
  } else {
    // Original reset mode: linear interpolation from initial to reset pose over 10s
    for (size_t i = 0; i < 7; ++i) {
      double cmd;
      if (elapsed_time_.toSec() > 10) {
        cmd = reset_pose_[i];
      } else {
        cmd = ((elapsed_time_.toSec()) / 10.0) * reset_pose_[i]
            + ((10 - elapsed_time_.toSec()) / 10.0) * initial_pose_[i];
      }
      position_joint_handles_[i].setCommand(cmd);

      // Keep in sync so streaming mode starts from the right state
      commanded_positions_[i] = cmd;
      commanded_velocities_[i] = 0.0;
    }
  }
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::JointPositionController,
                       controller_interface::ControllerBase)
