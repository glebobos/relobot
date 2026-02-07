// Copyright 2025 ReloBot Contributors
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
 
#include "diff_drive_hardware/diff_drive_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <thread>
#include <dirent.h>  // Include for directory operations

namespace diff_drive_hardware
{

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
hardware_interface::CallbackReturn DiffDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize storage for hardware states
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Initialize serial port
  if (!reconnect_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}
#pragma GCC diagnostic pop

bool DiffDriveHardware::reconnect_serial()
{
  std::string port = "/dev/ttyACM2";

  try {
    if (serial_port_.IsOpen()) {
      serial_port_.Close();
    }

    serial_port_.Open(port);
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    serial_port_.FlushIOBuffers();

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Successfully connected to %s", port.c_str());
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Failed to connect to %s: %s", port.c_str(), e.what());
  }

  return false;
}

bool DiffDriveHardware::isValidEncoderValue(const std::string& value) {
  try {
    double val = std::stod(value);
    return !std::isnan(val) && !std::isinf(val);
  } catch (...) {
    return false;
  }
}

hardware_interface::CallbackReturn DiffDriveHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Configuring ...please wait...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Starting controller ...");
  
  try {
    // First send reset command to clear encoder counts and stop motors
    std::string reset_command = "reset_counts\n";
    serial_port_.Write(reset_command);
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Sent encoder reset command");
    
    // Wait a bit for the reset to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Then send zero command as backup
    std::string zero_command = "0.00,0.00\n";
    serial_port_.Write(zero_command);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Error sending reset/zero command: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Stopping controller...");
  try {
    // Send reset command to clear encoder counts and stop motors
    std::string reset_command = "reset_counts\n";
    serial_port_.Write(reset_command);
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Sent encoder reset command");
    
    // Wait a bit for the reset to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Then send zero command as backup
    std::string zero_command = "0.00,0.00\n";
    serial_port_.Write(zero_command);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Error sending reset/zero command: %s", e.what());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, "position", &hw_positions_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, "position", &hw_positions_[1]));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, "velocity", &hw_velocities_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, "velocity", &hw_velocities_[1]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, "velocity", &hw_commands_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, "velocity", &hw_commands_[1]));

  return command_interfaces;
}

// For the read() function:
hardware_interface::return_type DiffDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_.IsOpen()) {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveHardware"), 
                "Serial port disconnected, attempting to reconnect...");
    if (!reconnect_serial()) {
      return hardware_interface::return_type::ERROR;
    }
  }

  try {
    if (serial_port_.IsDataAvailable()) {
      std::string response;
      // Changed this line - using '\n' as line terminator
      serial_port_.ReadLine(response, '\n', 100);
      
      if (response.empty()) {
        return hardware_interface::return_type::OK;
      }

      std::stringstream ss(response);
      std::string left_str, right_str;
      
      if (std::getline(ss, left_str, ',') && std::getline(ss, right_str)) {
        if (isValidEncoderValue(left_str) && isValidEncoderValue(right_str)) {
          double left_encoder = std::stod(left_str);
          double right_encoder = std::stod(right_str);
          
          hw_positions_[0] = left_encoder;
          hw_positions_[1] = right_encoder;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("DiffDriveHardware"), 
                     "Invalid encoder values received: %s, %s", 
                     left_str.c_str(), right_str.c_str());
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), 
                 "Error reading from serial port: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// For the write() function:
hardware_interface::return_type DiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_.IsOpen()) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), 
                 "Serial port is not open");
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Validate commands
    if (hw_commands_.size() < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), 
                   "Invalid number of commands");
      return hardware_interface::return_type::ERROR;
    }

    std::stringstream command;
    command << std::fixed << std::setprecision(2) 
            << hw_commands_[0] << "," << hw_commands_[1] << "\n";
    
    std::string cmd_str = command.str();
    
    // Debug output
    RCLCPP_DEBUG(rclcpp::get_logger("DiffDriveHardware"), 
                 "Writing to serial: %s", cmd_str.c_str());

    // Flush any existing data
    serial_port_.FlushIOBuffers();
    
    // Write the command
    serial_port_.Write(cmd_str);
    
    // Debug output for successful write
    RCLCPP_DEBUG(rclcpp::get_logger("DiffDriveHardware"), 
                 "Successfully wrote commands: [%f, %f]", 
                 hw_commands_[0], hw_commands_[1]);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), 
                 "Error writing to serial port: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace diff_drive_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diff_drive_hardware::DiffDriveHardware, hardware_interface::SystemInterface)