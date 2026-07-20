# Wheel Calibration Tool

This directory contains the utility script used to calibrate the feedforward control constants ($K$ and $C$) for the differential drive robot's wheels.

## How it Works

The calibration routine measures the motor speed constants by running sweeps in both forward and backward directions for the left and right wheels independently:
1. **Sweep**: The script applies duty cycle inputs (PWM values: `[0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.32]` and negative equivalents) directly to the motors via the ROS2 topic `/robot_joint_commands`.
2. **Measure**: For each input, it waits for the motor speed to stabilize (1.5 seconds) and then samples the steady-state wheel velocity (from `/robot_joint_states`) for 2.0 seconds.
3. **Fit**: It performs a linear regression fit using the equation $|U_{\text{pwm}}| = K \cdot |\omega| + C$ to find the feedforward gain ($K$) and stiction offset ($C$) for both directions of both wheels.
4. **Output**: It prints the formatted C++ `#define` code containing the empirical constants, ready to copy-paste into `main.cpp`.

---

## Calibration Steps

### Recommended: Automated One-Command Calibration
Run the automated launcher script from the root of the repository:
```bash
./start_wheel_calibration.sh
```
This script will automatically:
1. Stop running robot containers to avoid command interference.
2. Build and flash `pico_wheels_calibration.uf2` to the wheels board.
3. Start the micro-ROS agent and execute the PWM sweep calibration routine.

---

### Manual Step-by-Step Procedure

If you prefer to run steps individually:

1. **Flash Calibration Firmware**:
   ```bash
   ./run_firmware.sh flash wheels_calibration
   ```

2. **Run Calibration Sweep**:
   ```bash
   docker compose -f ros2_ws/docker-compose.yml run --rm ros2_diff_robot bash -c "source /opt/ros/humble/setup.bash && source /uros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent multiserial --devs \"/dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyACM3\" -b 115200 & sleep 5 && python3 /ros2_ws/helpers/wheel_calibration/calibrate.py"
   ```

> [!WARNING]
> Keep the robot in a safe environment or lift the wheels off the ground, as the wheels will rotate during calibration sweeps.

### 3. Apply the Coefficients
1. Copy the printed `#define` lines from the calibration script output.
2. Replace the old constants in [main.cpp](file:///home/admin/projects/relobot/pico_ware_wheels_microros/src/main.cpp#L31-L38):
   ```cpp
   // --- Feedforward: pwm_fraction = K * |omega_rad_s| + C ---
   #define FF_K_LEFT_FWD               ...
   #define FF_C_LEFT_FWD               ...
   ...
   ```
3. Rebuild standard wheels firmware:
   ```bash
   ./run_firmware.sh build wheels
   ```
4. Flash standard firmware back onto the wheels microcontroller:
   ```bash
   ./run_firmware.sh flash wheels
   ```

---

## Diagnostics: Showing Raw Ticks and Speeds

If you suspect an encoder is damaged, disconnected, or picking up noise, you can run the diagnostic script to print raw tick counts and velocities in real-time.

1. Start the micro-ROS agent in one terminal:
   ```bash
   docker compose -f ros2_ws/docker-compose.yml run --rm ros2_diff_robot bash -c "source /opt/ros/humble/setup.bash && source /uros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent multiserial --devs \"/dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyACM3\" -b 115200"
   ```
2. In another terminal, run the diagnostic script:
   ```bash
docker compose -f ros2_ws/docker-compose.yml exec ros2_diff_robot bash -c "source /opt/ros/humble/setup.bash && python3 /ros2_ws/helpers/wheel_calibration/show_ticks.py"

   ```
3. Manually turn the wheels. You should see the tick counts increase/decrease smoothly. If a wheel's ticks increase rapidly on their own when the wheel is stationary or spin is initiated, that indicates EMI/noise on that channel.

