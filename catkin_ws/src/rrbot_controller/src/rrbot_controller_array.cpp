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

#include "rrbot_controller/rrbot_controller_array.hpp"

#include <string>
#include <vector>

namespace rrbot_controller
{

bool RRBotControllerArray::init(
  hardware_interface::PositionJointInterface * hw, ros::NodeHandle & nh)
{
  // List of controlled joints
  std::string param_name = "joints";
  if(!nh.getParam(param_name, joint_names_))
  {
    ROS_ERROR_STREAM_NAMED("RRBotControllerArray",
      "Failed to getParam '" << param_name <<"' (namespace: " << nh.getNamespace() << ").");
    return false;
  }
  size_t num_joints = joint_names_.size();

  if (num_joints == 0) {
    ROS_ERROR_STREAM_NAMED("RRBotControllerArray", "List of joint names is empty.");
    return false;
  }
  for(size_t i = 0; i < num_joints; ++i)
  {
    try
    {
      joints_.push_back(hw->getHandle(joint_names_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM_NAMED("RRBotControllerArray", "Exception thrown: " << e.what());
      return false;
    }
  }

  // State publisher
  state_publisher_.reset(new ControllerStatePublisher(nh, "state", 1));

  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = joint_names_[0];
  state_publisher_->unlock();

  commands_buffer_.writeFromNonRT(std::vector<double>(num_joints, 0.0));

  command_subscriber_ = nh.subscribe<ControllerCommandMsg>(
    "command", 1, &RRBotControllerArray::commandCB, this);
  return true;
}

void RRBotControllerArray::starting(const ros::Time & /*time*/)
{
  // Start controller with current joint positions
  std::vector<double> & commands = *commands_buffer_.readFromRT();
  for(size_t i = 0; i < joint_names_.size(); ++i)
  {
    commands[i] = joints_[i].getPosition();
  }
}

void RRBotControllerArray::update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
  std::vector<double> & commands = *commands_buffer_.readFromRT();
  for(size_t i = 0; i < joint_names_.size(); ++i) {
    joints_[i].setCommand(commands[i]);
  }

  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = ros::Time::now();
    state_publisher_->msg_.set_point = joints_[0].getPosition();

    state_publisher_->unlockAndPublish();
  }
}

void RRBotControllerArray::commandCB(const ControllerCommandMsg::ConstPtr & msg)
{
  if (msg->data.size() != joint_names_.size())
  {
    ROS_ERROR_STREAM_NAMED("RRBotControllerArray",
      "Dimension of command (" << msg->data.size() << ") does not match number of joints ("
      << joint_names_.size() << ")! Not executing!");
    return;
  }
  commands_buffer_.writeFromNonRT(msg->data);
}

}  // namespace rrbot_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rrbot_controller::RRBotControllerArray, controller_interface::ControllerBase)
