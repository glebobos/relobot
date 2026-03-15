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

#include <algorithm>
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
  prev_positions_.resize(info_.joints.size(), 0.0);
  first_read_ = true;

  // Initialize PID controllers from hardware parameters
  {
    auto get_hw_param = [&](const std::string & name, double default_val) -> double {
      auto it = info_.hardware_parameters.find(name);
      return it != info_.hardware_parameters.end() ? std::stod(it->second) : default_val;
    };

    double lp   = get_hw_param("left_pid_p",   0.5);
    double li   = get_hw_param("left_pid_i",   0.1);
    double ld   = get_hw_param("left_pid_d",   0.01);
    double lmax = get_hw_param("left_i_max",   2.0);
    double lmin = get_hw_param("left_i_min",  -2.0);

    double rp   = get_hw_param("right_pid_p",   0.5);
    double ri   = get_hw_param("right_pid_i",   0.1);
    double rd   = get_hw_param("right_pid_d",   0.01);
    double rmax = get_hw_param("right_i_max",   2.0);
    double rmin = get_hw_param("right_i_min",  -2.0);

    pid_left_.initPid(lp, li, ld, lmax, lmin, true);
    pid_right_.initPid(rp, ri, rd, rmax, rmin, true);

    RCLCPP_INFO(
      rclcpp::get_logger("DiffDriveHardware"),
      "PID left [p=%.3f i=%.3f d=%.3f imax=%.2f imin=%.2f], "
      "right [p=%.3f i=%.3f d=%.3f imax=%.2f imin=%.2f]",
      lp, li, ld, lmax, lmin, rp, ri, rd, rmax, rmin);
  }

  // Initialize serial port
  if (!reconnect_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool DiffDriveHardware::reconnect_serial()
{
  const char* env_port = std::getenv("DIFF_DRIVE_TTY");
  std::string port = env_port ? env_port : "/dev/ttyACM2";

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

  pid_left_.reset();
  pid_right_.reset();
  std::fill(prev_positions_.begin(), prev_positions_.end(), 0.0);
  prev_cmd_sent_[0] = 0.0;
  prev_cmd_sent_[1] = 0.0;
  stall_logged_ = false;
  stall_cycles_[0] = stall_cycles_[1] = 0;
  stall_boost_[0]  = stall_boost_[1]  = 0.0;
  encoder_age_[0]  = encoder_age_[1]  = 0;
  first_read_ = true;

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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
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

          // Estimate angular velocities from position deltas, then apply an
          // exponential moving average (EMA) to suppress the tick noise that
          // coarse encoders (60 cpr) produce at low speed.  At 30 Hz a single
          // encoder tick looks like a 1–2 rad/s spike for one cycle; without
          // filtering P would amplify that directly into oscillation.
          // alpha=0.25: 75% previous, 25% new measurement.
          constexpr double VEL_ALPHA = 0.25;
          if (!first_read_ && period.seconds() > 0.0) {
            double raw_left  = (hw_positions_[0] - prev_positions_[0]) / period.seconds();
            double raw_right = (hw_positions_[1] - prev_positions_[1]) / period.seconds();
            hw_velocities_[0] = VEL_ALPHA * raw_left  + (1.0 - VEL_ALPHA) * hw_velocities_[0];
            hw_velocities_[1] = VEL_ALPHA * raw_right + (1.0 - VEL_ALPHA) * hw_velocities_[1];
          }
          prev_positions_[0] = hw_positions_[0];
          prev_positions_[1] = hw_positions_[1];
          // Position changed → both wheels got a fresh packet this cycle
          encoder_age_[0] = 0;
          encoder_age_[1] = 0;
          if (first_read_) {
            // First valid encoder packet: reset PIDs so integrators start
            // from zero with real velocity data, not the zero-velocity spike.
            pid_left_.reset();
            pid_right_.reset();
          }
          first_read_ = false;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("DiffDriveHardware"), 
                     "Invalid encoder values received: %s, %s", 
                     left_str.c_str(), right_str.c_str());
        }
      }
    } else {
      // No new serial data this cycle — Pico only sends when encoder counts change.
      // Increment age counters; velocity estimate is left unchanged (last known good).
      // Stall detection uses encoder_age_, not velocity magnitude, so it is immune
      // to the low packet rate at slow speed (one tick per ~9 cycles at 0.35 rad/s).
      encoder_age_[0]++;
      encoder_age_[1]++;
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
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

    constexpr double MAX_SPEED = 7.5;    // rad/s — matches Pico firmware constant
    // Only treat as truly stopped when commands are essentially zero.
    // Any larger value cuts off slow-turn commands that nav2/joystick sends.
    // The stale-velocity zeroing in read() already prevents jitter at rest.
    constexpr double CMD_DEAD_ZONE = 0.01; // rad/s
    double left_cmd  = hw_commands_[0];
    double right_cmd = hw_commands_[1];

    // When both wheels are commanded to stop, skip PID and reset integrators.
    // This prevents the integrator from accumulating error against a stale or
    // noisy velocity reading while the robot is stationary.
    const bool both_stopped =
      std::abs(hw_commands_[0]) < CMD_DEAD_ZONE &&
      std::abs(hw_commands_[1]) < CMD_DEAD_ZONE;

    if (both_stopped) {
      pid_left_.reset();
      pid_right_.reset();
      stall_cycles_[0] = stall_cycles_[1] = 0;
      stall_boost_[0]  = stall_boost_[1]  = 0.0;
      // Bypass slew limiter for immediate stop — if prev_cmd_sent_ held a
      // boosted value, the slew would keep sending nonzero to Pico for
      // several cycles causing persistent PWM whine after stopping.
      prev_cmd_sent_[0] = prev_cmd_sent_[1] = 0.0;
      left_cmd  = 0.0;
      right_cmd = 0.0;
    } else if (!first_read_) {
      // Apply PID velocity correction only after we have valid measurements.
      double left_error  = hw_commands_[0] - hw_velocities_[0];
      double right_error = hw_commands_[1] - hw_velocities_[1];
      double left_correction  = pid_left_.computeCommand(left_error,  period.nanoseconds());
      double right_correction = pid_right_.computeCommand(right_error, period.nanoseconds());
      left_cmd  = std::clamp(hw_commands_[0] + left_correction,  -MAX_SPEED, MAX_SPEED);
      right_cmd = std::clamp(hw_commands_[1] + right_correction, -MAX_SPEED, MAX_SPEED);
    }

    // ── Anti-slip: slew-rate limiter ───────────────────────────────────────
    constexpr double SLEW_RATE = 0.5;  // rad/s per control cycle
    left_cmd  = std::clamp(left_cmd,  prev_cmd_sent_[0] - SLEW_RATE, prev_cmd_sent_[0] + SLEW_RATE);
    right_cmd = std::clamp(right_cmd, prev_cmd_sent_[1] - SLEW_RATE, prev_cmd_sent_[1] + SLEW_RATE);
    // ──────────────────────────────────────────────────────────────────────

    // ── Motor deadband compensation ────────────────────────────────────────
    // The Pico's speed_to_pwm generates duty cycles too low to overcome motor
    // stiction at slow commands (e.g. 0.1 rad/s → ~4% PWM), causing coil whine
    // without rotation. Any nonzero command is raised to the minimum effective
    // speed. The threshold matches the ~8% PWM stall point back-calculated from
    // the Pico's speed_to_pwm coefficients: left≈0.57, right≈0.55 rad/s.
    constexpr double MIN_EFFECTIVE_SPEED = 0.35;  // rad/s
    auto apply_deadband = [&](double cmd) -> double {
      if (cmd > CMD_DEAD_ZONE && cmd < MIN_EFFECTIVE_SPEED)
        return MIN_EFFECTIVE_SPEED;
      if (cmd < -CMD_DEAD_ZONE && cmd > -MIN_EFFECTIVE_SPEED)
        return -MIN_EFFECTIVE_SPEED;
      return cmd;
    };
    left_cmd  = apply_deadband(left_cmd);
    right_cmd = apply_deadband(right_cmd);
    // ──────────────────────────────────────────────────────────────────────

    // ── Stall-ramp accumulator ─────────────────────────────────────────────
    // Uses encoder_age_ (cycles since last packet) instead of velocity magnitude.
    // At 0.35 rad/s with 60 CPR the Pico sends one packet every ~9 cycles, so
    // velocity would decay below any reasonable threshold between packets. Age
    // is immune to this: if a packet arrived within MOVING_AGE_MAX cycles the
    // wheel is considered moving regardless of estimated velocity.
    // Stall detection only runs for low-speed commands — large commands with
    // vel=0 are normal slew/startup lag, not real stalls.
    constexpr int    MOVING_AGE_MAX        = 20;   // cycles (~0.67 s) — wheel moving if packet arrived within this
    constexpr int    STALL_CYCLES_THRESH   = 15;   // ~0.5 s at 30 Hz before first boost
    constexpr double STALL_RAMP_STEP       = 0.20; // rad/s per threshold crossing
    constexpr double STALL_BOOST_MAX       = 3.5;  // rad/s max extra push
    constexpr double STALL_DETECT_MAX_CMD  = 1.0;  // rad/s — ignore stall above this
    for (int w = 0; w < 2; ++w) {
      double & cmd   = (w == 0) ? left_cmd  : right_cmd;
      double   hcmd  = hw_commands_[w];
      const bool wheel_moving  = (encoder_age_[w] < MOVING_AGE_MAX);
      const bool low_speed_cmd = std::abs(hcmd) > CMD_DEAD_ZONE &&
                                 std::abs(hcmd) < STALL_DETECT_MAX_CMD;
      if (!wheel_moving && low_speed_cmd) {
        stall_cycles_[w]++;
        if (stall_cycles_[w] >= STALL_CYCLES_THRESH) {
          stall_boost_[w] = std::min(stall_boost_[w] + STALL_RAMP_STEP, STALL_BOOST_MAX);
          stall_cycles_[w] = 0;
          RCLCPP_DEBUG(
            rclcpp::get_logger("DiffDriveHardware"),
            "Stall boost wheel %d: boost=%.2f sent_before=%.3f", w, stall_boost_[w], cmd);
        }
      } else if (wheel_moving) {
        // Wheel is moving — clear boost so normal PID takes over.
        // Also anchor prev_cmd_sent_ to the unbosted command level so the
        // slew limiter doesn't stay locked to the high boosted value,
        // which would cause residual PWM after the obstacle is cleared.
        if (stall_boost_[w] > 0.0) {
          RCLCPP_DEBUG(
            rclcpp::get_logger("DiffDriveHardware"),
            "Stall cleared wheel %d: age=%d boost was %.2f", w, encoder_age_[w], stall_boost_[w]);
          prev_cmd_sent_[w] = std::copysign(std::abs(hw_commands_[w]), cmd);
        }
        stall_cycles_[w] = 0;
        stall_boost_[w]  = 0.0;
      }
      // Apply accumulated boost (safe even when boost==0)
      if (stall_boost_[w] > 0.0) {
        cmd = std::copysign(std::abs(cmd) + stall_boost_[w], cmd);
        cmd = std::clamp(cmd, -MAX_SPEED, MAX_SPEED);
      }
    }
    // ──────────────────────────────────────────────────────────────────────

    prev_cmd_sent_[0] = left_cmd;
    prev_cmd_sent_[1] = right_cmd;

    std::stringstream command;
    command << std::fixed << std::setprecision(2)
            << left_cmd << "," << right_cmd << "\n";

    std::string cmd_str = command.str();
    
    // Debug output
    RCLCPP_DEBUG(rclcpp::get_logger("DiffDriveHardware"), 
                 "Writing to serial: %s", cmd_str.c_str());

    // Flush any existing data
    serial_port_.FlushIOBuffers();
    
    // Write the command
    serial_port_.Write(cmd_str);
    
    // Log once when a wheel is commanded but stalled (vel==0)
    const bool left_stalled  = std::abs(hw_commands_[0]) > CMD_DEAD_ZONE && std::abs(hw_velocities_[0]) < 0.01;
    const bool right_stalled = std::abs(hw_commands_[1]) > CMD_DEAD_ZONE && std::abs(hw_velocities_[1]) < 0.01;
    if ((left_stalled || right_stalled) && !stall_logged_) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("DiffDriveHardware"),
        "STALL: cmd=[%.3f, %.3f] vel=[%.3f, %.3f] sent=[%.3f, %.3f] err=[%.3f, %.3f]",
        hw_commands_[0], hw_commands_[1],
        hw_velocities_[0], hw_velocities_[1],
        left_cmd, right_cmd,
        hw_commands_[0] - hw_velocities_[0], hw_commands_[1] - hw_velocities_[1]);
      stall_logged_ = true;
    } else if (!left_stalled && !right_stalled) {
      stall_logged_ = false;  // reset so next stall is logged again
    }
    
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