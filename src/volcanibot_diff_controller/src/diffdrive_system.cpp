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

#include "volcanibot_diff_controller/diffdrive_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Custom motor controller libs
#include "volcanibot_diff_controller/roboteq_comms.hpp"
#include "volcanibot_diff_controller/moons_comms.hpp"
#include "volcanibot_diff_controller/zlac8015_comms.hpp"

namespace volcanibot_diff_controller
{
  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.device_name = info_.hardware_parameters["device_name"];
    cfg_.port = info_.hardware_parameters["port"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);

    if (info_.hardware_parameters.count("gear_ratio"))
      gear_ratio_ = std::stod(info_.hardware_parameters["gear_ratio"]);
    else
      gear_ratio_ = 1.0;

    has_encoders_ = !(info_.hardware_parameters.count("has_encoders") &&
                      info_.hardware_parameters.at("has_encoders") == "false");

    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"),
    //     "Gear ratio set to %.2f, encoders %s", gear_ratio_,
    //     has_encoders_ ? "enabled" : "disabled");

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffDriveSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveSystemHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Map 4 URDF joints to 2 physical motors.
    // Joint ordering in xacro: [0]=front_left, [1]=front_right, [2]=rear_left, [3]=rear_right
    // Front-left and rear-left share left motor; front-right and rear-right share right motor.
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &front_left_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &front_left_velocity_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_POSITION, &front_right_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &front_right_velocity_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[2].name, hardware_interface::HW_IF_POSITION, &rear_left_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[2].name, hardware_interface::HW_IF_VELOCITY, &rear_left_velocity_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[3].name, hardware_interface::HW_IF_POSITION, &rear_right_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[3].name, hardware_interface::HW_IF_VELOCITY, &rear_right_velocity_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // 4 command interfaces — diff_drive_controller sends same value to both wheels per side
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &front_left_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &front_right_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[2].name, hardware_interface::HW_IF_VELOCITY, &rear_left_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[3].name, hardware_interface::HW_IF_VELOCITY, &rear_right_cmd_));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Configuring ...please wait...");

    if (cfg_.device_name == "roboteq")
      comm_ = new RoboteqComm();
    else if (cfg_.device_name == "moons")
      comm_ = new MoonsComm();
    else if (cfg_.device_name == "zlac8015")
      comm_ = new ZLAC8015Comm();

    bool success = comm_->ConnectComm(cfg_.port, cfg_.baud_rate, cfg_.timeout_ms);

    if (success)
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully configured!");

    return success ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
  }

  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Cleaning up ...please wait...");
    comm_->DisconnectComm();
    delete comm_;
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Activating ...please wait...");

    // set some default values
    left_motor_velocity_ = 0.0;
    right_motor_velocity_ = 0.0;
    left_motor_position_ = 0.0;
    right_motor_position_ = 0.0;

    front_left_position_ = rear_left_position_ = 0.0;
    front_right_position_ = rear_right_position_ = 0.0;
    front_left_velocity_ = rear_left_velocity_ = 0.0;
    front_right_velocity_ = rear_right_velocity_ = 0.0;
    front_left_cmd_ = rear_left_cmd_ = 0.0;
    front_right_cmd_ = rear_right_cmd_ = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffDriveSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!comm_->connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    if (has_encoders_) {
      double left_rpm = 0.0, right_rpm = 0.0;
      bool success = comm_->ReadRPM(left_rpm, right_rpm);

      if (success) {
        const double RPM_TO_RAD_S = (2.0 * M_PI) / 60.0;
        left_motor_velocity_  = -left_rpm  * RPM_TO_RAD_S / gear_ratio_;
        right_motor_velocity_ = -right_rpm * RPM_TO_RAD_S / gear_ratio_;

        left_motor_position_  += period.seconds() * left_motor_velocity_;
        right_motor_position_ += period.seconds() * right_motor_velocity_;

        front_left_velocity_  = rear_left_velocity_  = left_motor_velocity_;
        front_right_velocity_ = rear_right_velocity_ = right_motor_velocity_;
        front_left_position_  = rear_left_position_  = left_motor_position_;
        front_right_position_ = rear_right_position_ = right_motor_position_;
      }
      // On read timeout keep last velocity — don't return ERROR (would deactivate hardware)
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type volcanibot_diff_controller::DiffDriveSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Writing...");
    if (!comm_->connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // diff_drive_controller sends rad/s, Roboteq !S expects RPM
    // Use front joint commands (controller sends same value to front and rear on each side)
    const double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI);

    double left_rpm  = front_left_cmd_  * RAD_S_TO_RPM * gear_ratio_;
    double right_rpm = front_right_cmd_ * RAD_S_TO_RPM * gear_ratio_;

  // Apply deadband: if commanded but below minimum, clamp to minimum in same direction
    // if (std::abs(left_rpm) > 0.0 && std::abs(left_rpm) < MIN_RPM)
    //   left_rpm = std::copysign(MIN_RPM, left_rpm);

    // if (std::abs(right_rpm) > 0.0 && std::abs(right_rpm) < MIN_RPM)
    //   right_rpm = std::copysign(MIN_RPM, right_rpm);


    comm_->DriveCommand(-left_rpm, right_rpm);

    // RCLCPP_INFO(
    //     rclcpp::get_logger("DiffDriveSystemHardware"), "Got command %.5f for '%s'!", left_wheel_cmd_rpm_,
    //     info_.joints[0].name.c_str());
    // RCLCPP_INFO(
    //     rclcpp::get_logger("DiffDriveSystemHardware"), "Got command %.5f for '%s'!", right_wheel_cmd_rpm_,
    //     info_.joints[1].name.c_str());

    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Joints successfully written!");
    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"),
    //     "CMD rad/s L=%.2f R=%.2f → RPM L=%.1f R=%.1f",
    //     front_left_cmd_, front_right_cmd_, left_rpm, right_rpm);

        return hardware_interface::return_type::OK;
      }

} // namespace volcanibot_diff_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    volcanibot_diff_controller::DiffDriveSystemHardware, hardware_interface::SystemInterface)
