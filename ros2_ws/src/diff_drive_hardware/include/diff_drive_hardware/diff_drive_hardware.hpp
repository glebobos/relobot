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

#ifndef DIFF_DRIVE_HARDWARE__DIFF_DRIVE_HARDWARE_HPP_
#define DIFF_DRIVE_HARDWARE__DIFF_DRIVE_HARDWARE_HPP_

#include <string>
#include <vector>
#include <libserial/SerialPort.h>

#include "control_toolbox/pid.hpp"
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

  // PID controllers (one per wheel)
  control_toolbox::Pid pid_left_;
  control_toolbox::Pid pid_right_;

  // Previous positions for velocity estimation
  std::vector<double> prev_positions_;
  bool first_read_{true};

  // Last commands actually sent to the Pico (for slew-rate / anti-slip)
  double prev_cmd_sent_[2] = {0.0, 0.0};

  // Stall detection logging (log once per stall event)
  bool stall_logged_{false};

  // Stall-ramp accumulator: independent of PID, ramps up command when a wheel
  // is commanded but not moving (obstacle recovery)
  int stall_cycles_[2]   = {0, 0};    // consecutive cycles with vel==0 under command
  double stall_boost_[2] = {0.0, 0.0}; // extra rad/s added to overcome stall

  // Packet-age counters: cycles since last encoder update per wheel.
  // Used instead of velocity magnitude for stall detection — immune to the
  // low packet rate at slow speed (60 cpr, so one tick per ~9 cycles at 0.35 rad/s).
  int encoder_age_[2] = {0, 0};
};

}  // namespace diff_drive_hardware

#endif  // DIFF_DRIVE_HARDWARE__DIFF_DRIVE_HARDWARE_HPP_