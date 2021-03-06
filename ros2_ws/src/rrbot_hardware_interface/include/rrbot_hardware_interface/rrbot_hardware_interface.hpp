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

#ifndef RRBOT_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_
#define RRBOT_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "rrbot_hardware_interface/visibility_control.h"

namespace rrbot_hardware_interface
{
class RRBotHardwareInterface
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotHardwareInterface)

  RRBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  RRBOT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RRBOT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RRBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  RRBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  RRBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  RRBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace rrbot_hardware_interface

#endif  // RRBOT_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_
