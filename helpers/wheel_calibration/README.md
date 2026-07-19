# Wheel Calibration Tool

This directory contains the utility script used to calibrate the feedforward control constants ($K$ and $C$) for the differential drive robot's wheels.

## How it Works

The calibration routine measures the motor speed constants by running sweeps in both forward and backward directions for the left and right wheels independently:
1. **Sweep**: The script applies duty cycle inputs (PWM values: `[0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.32]` and negative equivalents) directly to the motors via the ROS2 topic `/robot_joint_commands`.
2. **Measure**: For each input, it waits for the motor speed to stabilize (1.5 seconds) and then samples the steady-state wheel velocity (from `/robot_joint_states`) for 1.0 second.
3. **Fit**: It performs a linear regression fit using the equation $|U_{\text{pwm}}| = K \cdot |\omega| + C$ to find the feedforward gain ($K$) and stiction offset ($C$) for both directions of both wheels.
4. **Output**: It prints the formatted C++ `#define` code containing the empirical constants, ready to copy-paste into `main.cpp`.

---

## Calibration Steps

### 1. Flash Calibration Firmware
Flash the calibration binary output `pico_wheels_calibration.uf2` to the wheels RP2040 microcontroller board.
- The binary is located at: `pico_ware_wheels_microros/src/pico_wheels_calibration.uf2`
- *Note for WSL2 users: You must attach the USB device to WSL2 using `usbipd-win` before it will be visible. See [FLASHING.md](file:///home/glebobos/projects/relobot/FLASHING.md#5-wsl2-usb-passthrough-usbipd-win) for details.*


### 2. Run the Calibration Node (Non-interfering Mode)
To calibrate the feedforward constants without interference from the main robot controller (which regularly publishes conflicting `0.0` command signals to hold position), you should run the micro-ROS agent and the calibration script together in a standalone container:

```bash
docker compose -f ros2_ws/docker-compose.yml run --rm ros2_diff_robot bash -c "source /opt/ros/humble/setup.bash && source /uros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent multiserial --devs \"/dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyACM3\" -b 115200 & sleep 5 && python3 /ros2_ws/helpers/wheel_calibration/calibrate.py"
```

> [!WARNING]
> Keep the robot in a safe environment or lift the wheels off the ground, as the wheels will rotate during calibration sweeps.


### 4. Apply the Coefficients
1. Copy the printed `#define` lines from the calibration script output.
2. Replace the old constants in [main.cpp](file:///home/glebobos/projects/relobot/pico_ware_wheels_microros/src/main.cpp#L31-L38):
   ```cpp
   // --- Feedforward: pwm_fraction = K * |omega_rad_s| + C ---
   #define FF_K_LEFT_FWD               ...
   #define FF_C_LEFT_FWD               ...
   ...
   ```
3. Rebuild the firmware:
   ```bash
   docker compose -f pico_ware_wheels_microros/docker-compose.yml run --rm builder
   ```
4. Flash the final standard firmware `pico_wheels_microros.uf2` back onto the microcontroller.
