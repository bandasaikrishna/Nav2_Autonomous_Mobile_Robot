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

#include "agv_proto/robot_hardware_interface_agv.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agv_proto_hardware
{
hardware_interface::return_type DiffBotSystemHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }
  
  
  
  std::vector<std::string> target_sn;
  std::vector<std::string> target_cfg;
  std::vector<odrive_endpoint *> endpoint;
  std::vector<Json::Value> json;
  
  std::string od_sn="0x347B36623031";
  //this->get_parameter_or<std::string>("od_sn", od_sn, "0x347B36623031");

  // parse sn string for comma-separated values / more than one targets
  std::stringstream ssn(od_sn);
  while (ssn.good()) {
     std::string substr;
     getline(ssn, substr, ',');
     target_sn.push_back(substr);
  }

  // Initialize publisher/subscriber for each target
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "%d odrive instances:", (int)target_sn.size());
  for (size_t i = 0; i < target_sn.size(); i++) {
     RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "- Instance %d: SN %s",
           static_cast<int>(i), target_sn.at(i).c_str());
    // Get odrive endpoint instance
    endpoint.push_back(new odrive_endpoint());

    // Enumerate Odrive target
    if (endpoint.at(i)->init(stoull(target_sn.at(i), 0, 16))) {
	RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "* Device not found!");
	return hardware_interface::return_type::ERROR;
    }

    // Read JSON from target
    Json::Value odrive_json;
    if (getJson(endpoint.at(i), &odrive_json)) {
	return hardware_interface::return_type::ERROR;
    }

    json.push_back(odrive_json);
  }

  uint32_t u32val = 8;

  endpoint_ = endpoint.at(0);
  odrive_json_ = json.at(0);

  writeOdriveData(endpoint_, odrive_json_, "axis0.requested_state", u32val);

  writeOdriveData(endpoint_, odrive_json_, "axis1.requested_state", u32val);

  

  readOdriveData(endpoint_, odrive_json_, "axis0.encoder.pos_estimate_counts", old_left_wheel_enc_cnt);
  readOdriveData(endpoint_, odrive_json_, "axis1.encoder.pos_estimate_counts", old_right_wheel_enc_cnt);


  // Example loop - reading values and updating motor velocity
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Starting idle loop, old_right_wheel_enc_cnt= %f, old_left_wheel_enc_cnt= %f",old_right_wheel_enc_cnt,old_left_wheel_enc_cnt);


  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type DiffBotSystemHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Starting ...please wait...");

  /*for (auto i = 0; i <= hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }*/

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Stopping ...please wait...");
  
  endpoint_->remove();
  delete endpoint_;

  /*for (auto i = 0; i <= hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }*/

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::read()
{
  //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Reading...");

    readOdriveData(endpoint_, odrive_json_, std::string("axis1.encoder.pos_estimate_counts"), right_wheel_enc_cnt);
    right_motor_pos = right_motor_pos + ((right_wheel_enc_cnt- old_right_wheel_enc_cnt)/90)*360 ;
    hw_positions_[0]=(angles::from_degrees(right_motor_pos));
    old_right_wheel_enc_cnt = right_wheel_enc_cnt;
       
    readOdriveData(endpoint_, odrive_json_, std::string("axis0.encoder.pos_estimate_counts"), left_wheel_enc_cnt);
    left_motor_pos =(left_motor_pos + (((left_wheel_enc_cnt - old_left_wheel_enc_cnt )/90)*360));
    hw_positions_[1]=-(angles::from_degrees(left_motor_pos));
    old_left_wheel_enc_cnt = left_wheel_enc_cnt;
    
    //ROS_INFO("right wheel = %f  left wheel = %f", joint_position_[0], joint_position_[1]);
    //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "right wheel = %f  left wheel = %f", hw_positions_[0], hw_positions_[1]);
    float left_vel, right_vel;
    readOdriveData(endpoint_, odrive_json_, std::string("axis1.encoder.vel_estimate"), right_vel);
    readOdriveData(endpoint_, odrive_json_, std::string("axis0.encoder.vel_estimate"), left_vel);
    //if(right_vel != 0 || left_vel != 0)
    	//RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "right wheel = %f  left wheel = %f", right_vel, left_vel);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type agv_proto_hardware::DiffBotSystemHardware::write()
{
  //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");

  float velocity;
    //RCLCPP_INFO(
      //rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      //info_.joints[i].name.c_str());
      
    /*for (auto i = 0u; i < hw_commands_.size(); i++)
    {
      // Simulate sending commands to the hardware
      RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());

      //hw_velocities_[i] = hw_commands_[i];
    }*/
    
    
    
    velocity=(hw_commands_[0])/6.28319;
    
    
    
    //if(right_prev_cmd!=velocity)
    //{
	    writeOdriveData(endpoint_, odrive_json_, "axis1.controller.input_vel", velocity);
	    right_prev_cmd=velocity;
    //}
    
    velocity=-(hw_commands_[1])/6.28319;
    
    //if(left_prev_cmd!=velocity)
    //{
	    writeOdriveData(endpoint_, odrive_json_, "axis0.controller.input_vel", velocity);
	    left_prev_cmd=velocity;
    //}

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  agv_proto_hardware::DiffBotSystemHardware, hardware_interface::SystemInterface)
