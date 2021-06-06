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

#ifndef RRBOT_CONTROLLER__RRBOT_CONTROLLER_ARRAY_HPP_
#define RRBOT_CONTROLLER__RRBOT_CONTROLLER_ARRAY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/joint_command_interface.h"
#include "controller_interface/controller.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/node_handle.h"

#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64MultiArray.h"

namespace rrbot_controller
{
class RRBotControllerArray :
public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::PositionJointInterface * hw, ros::NodeHandle & nh);

  void starting(const ros::Time & time);

  void update(const ros::Time & time, const ros::Duration & period);

protected:
  std::vector<std::string> joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;

  using ControllerCommandMsg = std_msgs::Float64MultiArray;

  ros::Subscriber command_subscriber_;
  void commandCB(const ControllerCommandMsg::ConstPtr & msg);

  using ControllerStateMsg = control_msgs::JointControllerState;
  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  std::unique_ptr<ControllerStatePublisher> state_publisher_;
};

}  // namespace rrbot_controller

#endif  // RRBOT_CONTROLLER__RRBOT_CONTROLLER_ARRAY_HPP_
