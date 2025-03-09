#ifndef DIFF_DRIVE_HARDWARE__DIFF_DRIVE_HARDWARE_HPP_
#define DIFF_DRIVE_HARDWARE__DIFF_DRIVE_HARDWARE_HPP_

#include <string>
#include <vector>
#include <libserial/SerialPort.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace diff_drive_hardware
{
class DiffDriveHardware : public hardware_interface::SystemInterface
{
public:
  DiffDriveHardware() = default;
  ~DiffDriveHardware() = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool reconnect_serial();
  bool isValidEncoderValue(const std::string& value);

  // Constants
  static constexpr auto SERIAL_TIMEOUT = std::chrono::milliseconds(100);
  static constexpr auto READ_TIMEOUT = std::chrono::milliseconds(10);

  // Parameters
  std::string responce_name_ = "wheels\n";

  // Serial communication
  LibSerial::SerialPort serial_port_;

  // Store hardware states
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
};

}  // namespace diff_drive_hardware

#endif  // DIFF_DRIVE_HARDWARE__DIFF_DRIVE_HARDWARE_HPP_