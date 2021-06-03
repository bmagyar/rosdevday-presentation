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

#include "controller_manager/controller_manager.h"
#include "rrbot_hardware_interface/rrbot_hardware_interface.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrbot_hardware_interface");

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::NodeHandle root_nh;
  ros::NodeHandle robot_nh("~");

  rrbot_hardware_interface::RRBotHardwareInterface rrbot_hardware_interface;
  controller_manager::ControllerManager controller_manager(&rrbot_hardware_interface, root_nh);

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  rrbot_hardware_interface.init(root_nh, robot_nh);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    // Receive current state from robot
    if (!rrbot_hardware_interface.read(timestamp, period)) {
      ROS_FATAL_NAMED("rrbot_hardware_interface", "Failed to read state from robot. Shutting down!");
      ros::shutdown();
    }

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;


    // Update the controllers
    controller_manager.update(timestamp, period);

    // Send new setpoint to robot
    rrbot_hardware_interface.write(timestamp, period);

    loop_rate.sleep();
  }

  spinner.stop();
  ROS_INFO_NAMED("rrbot_hardware_interface", "Shutting down.");

  return 0;
}
