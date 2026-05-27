// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIFFDRIVE_ROS2_CONTROL__DIFFDRIVE_SYSTEM_HPP_
#define DIFFDRIVE_ROS2_CONTROL__DIFFDRIVE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "volcanibot_diff_controller/visibility_control.h"
#include "volcanibot_diff_controller/base_comms.hpp"

namespace volcanibot_diff_controller
{
  class DiffDriveSystemHardware : public hardware_interface::SystemInterface
  {
  public:
    struct Config
    {
      std::string device_name = "";
      std::string port = "";
      int baud_rate = 0;
      int timeout_ms = 0;
      float loop_rate = 0.0;
    };

    RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveSystemHardware)

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    DIFFDRIVE_ROS2_CONTROL_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // serial communication obj
    BaseComm *comm_;
    Config cfg_;
    double gear_ratio_ = 1.0;  // motor-to-wheel gear ratio
    bool has_encoders_ = true;  // false → skip ReadRPM to avoid ERROR on timeout

    // Motor-level values (2 physical motors)
    double left_motor_velocity_;   // rad/s from left motor encoder
    double right_motor_velocity_;  // rad/s from right motor encoder
    double left_motor_position_;   // integrated position from left motor
    double right_motor_position_;  // integrated position from right motor

    // Per-joint state interfaces (4 joints, front/rear share same motor)
    double front_left_position_, rear_left_position_;
    double front_right_position_, rear_right_position_;
    double front_left_velocity_, rear_left_velocity_;
    double front_right_velocity_, rear_right_velocity_;

    // Per-joint command interfaces (4 joints)
    double front_left_cmd_, rear_left_cmd_;
    double front_right_cmd_, rear_right_cmd_;
  };

} // namespace volcanibot_diff_controller

#endif // DIFFDRIVE_ROS2_CONTROL__DIFFDRIVE_SYSTEM_HPP_
