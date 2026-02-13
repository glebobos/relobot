# Camera Calibration Tool

This tool runs the ROS 2 camera calibration node in a Docker container, isolating dependencies while allowing GUI interaction via X11 forwarding.

## Prerequisites

- Docker and Docker Compose installed
- A printed checkerboard pattern (default expectation: 8x6 interior corners)
- A running camera node publishing images

## Installation

The tool is self-contained. The Docker image will be built automatically the first time you run the script.

## Usage

1. **Ensure your camera is running:**
   Make sure your camera node is active and publishing images.
   ```bash
   ros2 topic list
   ```

2. **Run the calibration tool:**
   ```bash
   ./start_camera_calibration.sh
   ```

   **Custom parameters:**
   You can specify the checkerboard size, square size, and image topic:
   ```bash
   ./start_camera_calibration.sh --size 7x9 --square 0.02 --topic /my_camera/image_raw
   ```
   
   - `--size`: Number of interior corners (cols x rows). Example: `8x6`
   - `--square`: Side length of a square in meters. Example: `0.02` (20mm)
   - `--topic`: The ROS 2 topic publishing the camera images.
   - `--camera_name`: Name of the camera (default: `camera`)

## Calibration Process

1. The GUI window will open showing the camera feed.
2. Hold the checkerboard in the camera's view.
3. Move the checkerboard to different positions (left, right, top, bottom, near, far) and orientations (tilt X, tilt Y, skew).
4. The bars on the right side of the window (X, Y, Size, Skew) will turn green as you collect enough data.
5. When the "CALIBRATE" button becomes active, click it. This may hang the window for a minute while processing.
6. Once calibration is done, click "SAVE" to save the calibration data.
7. Click "COMMIT" to send data to the camera info manager (if supported) or use the saved file.

The calibration data is saved to the `calibration_data` folder in the current directory (mounted to `/tmp` in the container). You will find `calibrationdata.tar.gz` there.

## Troubleshooting

- **No GUI appears:** Ensure you have X11 forwarding enabled on your host. If running remotely (SSH), use `ssh -X` or `ssh -Y`.
- **"Waiting for service..."**: Ensure your camera node is running and the topic name is correct.
