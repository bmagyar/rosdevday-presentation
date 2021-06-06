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

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

namespace rrbot_hardware_interface
{
class RRBotHardwareInterface : public hardware_interface::RobotHW
{
public:
  bool init(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh);

  bool read(const ros::Time time, const ros::Duration period);

  bool write(const ros::Time time, const ros::Duration period);

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_command_interface_;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;

  std::vector<std::string> joint_names_;
};

}  // namespace rrbot_hardware_interface

#endif  // RRBOT_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_
