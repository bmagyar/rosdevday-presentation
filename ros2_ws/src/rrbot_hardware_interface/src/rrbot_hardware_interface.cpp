// Copyright (c) 2021, Bence Magyar and Denis Stogl
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

#include "rrbot_hardware_interface/rrbot_hardware_interface.hpp"

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rrbot_hardware_interface
{
hardware_interface::return_type RRBotHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> RRBotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        // TODO(anyone): insert correct interfaces
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        // TODO(anyone): insert correct interfaces
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type RRBotHardwareInterface::start()
{
  // stat execution on hardware
  RCLCPP_INFO(rclcpp::get_logger("RrbotHardwareInterface"), "Starting...");

  // in this simple example reset state to initial positions
  for (auto i = 0u; info_.joints.size(); ++i) {
    hw_states_[i] = stod(info_.joints[i].parameters["initial_position"]);
    hw_commands_[i] = hw_states_[i];
  }

  status_ = hardware_interface::status::STARTED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotHardwareInterface::stop()
{
  // prepare the robot to stop receiving commands
  RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), "Stopping...");

  status_ = hardware_interface::status::STOPPED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotHardwareInterface::read()
{
  // read robot states from hardware, in this example print only
  RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), "Reading...");

  // write command to hardware, in this example do mirror command to states
  for (auto i = 0u; hw_states_.size(); ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotHardwareInterface"),
      "Got state %.2f for joint %d!", hw_states_[i], i);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotHardwareInterface::write()
{
  // write command to hardware, in this example do mirror command to states
  for (auto i = 0u; hw_commands_.size(); ++i) {
    hw_commands_[i] = hw_states_[i];
  }

  return hardware_interface::return_type::OK;
}

}  // namespace rrbot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rrbot_hardware_interface::RRBotHardwareInterface, hardware_interface::SystemInterface
)
