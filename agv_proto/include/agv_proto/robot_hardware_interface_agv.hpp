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

#ifndef ROS2_CONTROL_DEMO_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <angles/angles.h>


#include <libusb-1.0/libusb.h>
#include "ros_odrive_msg/msg/odrive_msg.hpp"
#include "ros_odrive_msg/msg/odrive_ctrl.hpp"
#include <fstream>


#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"

#include "agv_proto/odrive_endpoint.hpp"
#include "agv_proto/odrive_utils.hpp"
#include "agv_proto/odrive_enums.hpp"
#include <jsoncpp/json/json.h>

#include "agv_proto/visibility_control.h"

#define ODRIVE_OK    0
#define ODRIVE_ERROR 1

#define MAX_NR_OF_TARGETS 16


namespace agv_proto_hardware
{
class DiffBotSystemHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  AGV_PROTO_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  AGV_PROTO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  AGV_PROTO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  AGV_PROTO_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  AGV_PROTO_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  AGV_PROTO_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  AGV_PROTO_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Store the wheeled robot position
  double base_x_, base_y_, base_theta_;
  
  
  double left_motor_pos=0,right_motor_pos=0;
  float right_wheel_enc_cnt =0, left_wheel_enc_cnt =0, old_left_wheel_enc_cnt =0, old_right_wheel_enc_cnt =0;
  int right_prev_cmd=0, left_prev_cmd=0;
  odrive_endpoint *endpoint_;
  Json::Value odrive_json_;
  
  
};

}  // namespace ros2_control_demo_hardware

#endif  // ROS2_CONTROL_DEMO_HARDWARE__DIFFBOT_SYSTEM_HPP_
