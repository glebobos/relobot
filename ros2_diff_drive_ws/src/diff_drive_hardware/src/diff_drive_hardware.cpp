#include "diff_drive_hardware/diff_drive_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>

namespace diff_drive_hardware
{

hardware_interface::CallbackReturn DiffDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);

  // Initialize storage for hardware states
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Initialize serial port
  try {
    serial_port_.Open("/dev/ttyACM1");
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    
    // Wait for serial connection to stabilize
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Flush any existing data
    serial_port_.FlushIOBuffers();
    
  } catch (const LibSerial::OpenFailed& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Failed to open serial port: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
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
    std::string zero_command = "0.00,0.00\n";
    serial_port_.Write(zero_command);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Error sending zero command: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Stopping controller...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export position state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, "position", &hw_positions_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, "position", &hw_positions_[1]));

  // Export velocity state interfaces
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

hardware_interface::return_type DiffDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try {
    if (serial_port_.IsDataAvailable()) {
      std::string response;
      serial_port_.ReadLine(response);
      
      std::stringstream ss(response);
      std::string left_str, right_str;
      
      if (std::getline(ss, left_str, ',') && std::getline(ss, right_str)) {
        try {
          double left_encoder = std::stod(left_str);
          double right_encoder = std::stod(right_str);
          
          hw_positions_[0] = left_encoder * (2.0 * M_PI / encoder_ticks_per_revolution_);
          hw_positions_[1] = right_encoder * (2.0 * M_PI / encoder_ticks_per_revolution_);
          
        } catch (const std::exception& e) {
          RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Error parsing encoder values: %s", e.what());
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Error reading from serial port: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try {
    std::stringstream command;
    command << std::fixed << std::setprecision(2) 
            << hw_commands_[0] << "," << hw_commands_[1] << "\n";
    
    serial_port_.Write(command.str());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Error writing to serial port: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace diff_drive_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diff_drive_hardware::DiffDriveHardware, hardware_interface::SystemInterface)