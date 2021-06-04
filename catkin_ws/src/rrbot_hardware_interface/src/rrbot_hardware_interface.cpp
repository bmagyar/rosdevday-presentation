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

namespace rrbot_hardware_interface
{

bool RRBotHardwareInterface::init(ros::NodeHandle & /*root_nh*/, ros::NodeHandle & robot_hw_nh)
{
  if (!robot_hw_nh.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'joint_names' on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
    "'joint_names' on the parameter server.");
  }

  size_t num_joints = joint_names_.size();
  ROS_INFO_NAMED("RRBotHardwareInterface", "Found %zu joints.", num_joints);

  hw_position_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_position_commands_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());

  //Create ros_control interfaces
  for (size_t i = 0; i < num_joints; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(
      hardware_interface::JointStateHandle(
        joint_names_[i], &hw_position_states_[i], &hw_velocity_states_[i], &hw_effort_states_[i]));

    // Create joint position control interface
    position_command_interface_.registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &hw_position_commands_[i]));
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_command_interface_);

  // stat execution on hardware
  ROS_INFO_NAMED("RRBotHardwareInterface", "Starting...");

  // in this simple example reset state to initial positions
  for (size_t i = 0; i < num_joints; ++i){
    hw_position_states_[i] = 0.0; // INITIAL POSITION is ZERO
    hw_position_commands_[i] = hw_position_states_[i];
  }

  return true;
}

bool RRBotHardwareInterface::read(
  const ros::Time time, const ros::Duration period)
{
  // read robot states from hardware, in this example print only
  ROS_INFO_NAMED("RRBotHardwareInterface", "Reading...");

  // write command to hardware, in this example do mirror command to states
  for (size_t i = 0; i < hw_position_states_.size(); ++i) {
    ROS_INFO_NAMED("RRBotHardwareInterface",
                   "Got state %.2f for joint %zu!", hw_position_states_[i], i);
  }

  return true;
}

bool RRBotHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
  // write command to hardware, in this example do mirror command to states
  for (size_t i = 0; i < hw_position_commands_.size(); ++i) {
    hw_position_states_[i] = hw_position_states_[i] +
                             (hw_position_commands_[i] - hw_position_states_[i]) / 3.0;
  }

  return true;
}

}  // namespace rrbot_hardware_interface
