# Projeto_Final_RSDIS — ROS Line Follower Stack

This repository contains a ROS catkin workspace for a camera-based line follower robot with optional RGB color detection. It targets Raspberry Pi hardware with a Pi Camera for on-robot runs and can also be launched without motors for development on a desktop.

Core features:
- Image processing node extracts a normalized line tracking error from the Pi camera feed.
- Preview/PID-style controller converts error to `cmd_vel` velocity commands.
- Motor driver node drives a differential robot using WiringPi software PWM.
- Optional RGB detector adjusts the base speed dynamically based on detected color.
- Statistics node logs lap metrics to CSV and publishes periodic summaries.
- Launch files to run the whole stack, a no-motor configuration, or keyboard teleop.


## Repository Layout
- `catkin_ws/` — Catkin workspace
  - `src/error_management/` — Preview controller publishing `/cmd_vel` from `/error`
  - `src/image_processing/` — Line error extraction from camera images
  - `src/line_statistics/` — Lap/telemetry logging and summaries
  - `src/motor_control/` — Differential drive via WiringPi from `/cmd_vel`
  - `src/rgb_detector/` — HSV color detector publishing `/colour`
  - `src/rgb_management/` — Adjusts controller base speed from `/colour`
  - `src/system_launch/` — Launch files composing the system
  - `src/teleop/` — Minimal keyboard teleop publishing `/cmd_vel`
- Submodule: `catkin_ws/src/raspicam_node` (UbiquityRobotics) — Pi Camera driver

The workspace’s top-level CMake points to ROS Melodic (`/opt/ros/melodic/...`). The raspicam submodule tracks the `kinetic` branch but is compatible.


## Requirements
- ROS Melodic (Ubuntu 18.04) or compatible.
- Catkin tools (`catkin_make`).
- OpenCV + cv_bridge.
- Raspberry Pi (for on-robot runs) with camera enabled.
- WiringPi (for the motor driver; Raspberry Pi only).

ROS packages (typical):
- `ros-melodic-roscpp`, `ros-melodic-std-msgs`, `ros-melodic-geometry-msgs`, `ros-melodic-sensor-msgs`, `ros-melodic-cv-bridge`.

Non-ROS system deps:
- OpenCV dev libraries (installed automatically with `cv_bridge` on Ubuntu).
- WiringPi (`sudo apt install wiringpi`) on Raspberry Pi for `motor_control`.


## Setup
1) Clone and init the submodule:
   - `git submodule update --init --recursive`

2) Build the workspace:
   - `cd catkin_ws`
   - `rosdep install --from-paths src --ignore-src -r -y`
   - `catkin_make`
   - `source devel/setup.bash`

3) Raspberry Pi camera: enable the camera interface and verify the device works (per UbiquityRobotics raspicam_node docs).


## Launching
All launch files live in `catkin_ws/src/system_launch/launch`.

- Full robot (camera + controller + RGB + logging + motors):
  - `roslaunch system_launch line_follower_full.launch`

- Development (camera + controller + RGB + logging, no motor driver):
  - `roslaunch system_launch line_follower_no_motor.launch`

- Keyboard teleop + motor driver only:
  - `roslaunch system_launch line_follower_teleop.launch`

Common launch arguments (override with `name:=value`):
- Controller: `base_speed`, `speed_min`, `speed_max`, `angular_max`, `boost`, `kp`, `ki`, `kd`, `error_timeout`.
- Image processing crop and detection: `crop_x`, `crop_y`, `crop_w`, `crop_h`, `row_far_ratio`, `margin_px`, `min_gap_px`, `canny_low`, `canny_high`, `blur_ksize`, `blur_sigma`, `thr_low`, `thr_high`, `draw_debug`.
- RGB detector thresholds: `rgb_*` hue/sat/value parameters and crop.
- Statistics: `stats_error_topic`, `stats_cmd_topic`, `stats_enable_keyboard`, `stats_status_period`, `stats_log_file`, `stats_sample_log_dir`.

Defaults worth noting:
- Camera resolution is set by raspicam_node to 410x308 @ 30fps.
- Stats CSV defaults to `~/Line_lap_logs/line_laps.csv` in the full launch.


## System Architecture (Topics & Flow)
Camera → Processing → Control → Motors, plus side channels for color and logging:

1) `raspicam_node` (submodule)
   - Publishes: `/raspicam_node/image/compressed`

2) `image_processing` (C++)
   - Subscribes: `/raspicam_node/image/compressed`
   - Publishes: `/error` (`std_msgs/Float32`, normalized line error), `/Processed_Image` (debug image)

3) `error_management` (preview/PID controller)
   - Subscribes: `/error`
   - Publishes: `/cmd_vel` (`geometry_msgs/Twist`)
   - Parameters: monitors `~base_speed` at runtime; other gains are loaded at startup.

4) `motor_control` (Raspberry Pi only)
   - Subscribes: `/cmd_vel`
   - Drives motors using WiringPi soft PWM on pins:
     - Left dir: 28/29, Right dir: 22/23, PWM: Left 25, Right 26 (WiringPi numbering)

5) `rgb_detector`
   - Subscribes: `/raspicam_node/image/compressed`
   - Publishes: `/colour` (`std_msgs/String` — `R`, `G`, `B`, or `N`), `/processed_image_rgb` (debug)

6) `rgb_management`
   - Subscribes: `/colour`
   - Sets parameter: `target_param` (default `/error_management/base_speed`) to color-dependent speeds

7) `line_statistics`
   - Subscribes: `/error`, `/cmd_vel`
   - Publishes: `lap_statistics` (`std_msgs/String`)
   - Logs: CSV to `log_file` and per-lap samples to `sample_log_dir`

8) `teleop` (optional)
   - Publishes: `/cmd_vel` based on keys (see Teleop section)


## Nodes & Parameters

### image_processing
Purpose: Estimate lateral line error from a cropped camera ROI using blur → threshold → Canny edges and two-edge detection on a scanline.

Key params (private `~`):
- `crop_x`, `crop_y`, `crop_w`, `crop_h` — ROI rectangle (pixels).
- `row_far_ratio` — Scanline as fraction of ROI height.
- `margin_px` — Ignore edges close to image sides; `min_gap_px` — min separation between two edges.
- `canny_low`, `canny_high`, `blur_ksize`, `blur_sigma` — Preprocessing.
- `thr_low`, `thr_high` — RGB `inRange` threshold for mask creation.
- `draw_debug` — Publish `/Processed_Image` with overlays when true.

I/O:
- In: `/raspicam_node/image/compressed`
- Out: `/error` (`std_msgs/Float32` in approximately −1..+1), `/Processed_Image` (mono8)

### error_management (node name: `preview_controller`)
Purpose: Convert error to linear/angular velocity. Base speed can be boosted when error is small.

Params (private `~`): `base_speed`, `speed_min`, `speed_max`, `angular_max`, `boost`, `kp`, `ki`, `kd`, `error_timeout`.
- Dynamic: `base_speed` is read each control tick; others are read on startup.

I/O:
- In: `/error`
- Out: `/cmd_vel`

### motor_control
Purpose: Drive motors on a Raspberry Pi using WiringPi software PWM.

I/O:
- In: `/cmd_vel`

Hardware mapping (WiringPi numbering):
- Left dir: 28/29; Right dir: 22/23; PWM: Left 25, Right 26.

Notes: Requires WiringPi and Raspberry Pi GPIO; not used on desktop.

### rgb_detector
Purpose: Detect red/green/blue in an ROI using HSV thresholds.

Key params (private `~`, with `_rgb` suffix in launch):
- ROI and preprocessing analogous to `image_processing` (`crop_*_rgb`, `row_far_ratio_rgb`, etc.).
- HSV thresholds: `sat_min`, `val_min`, `red_hue_min/max`, `red_hue2_min/max`, `green_hue_min/max`, `blue_hue_min/max`.
- `draw_debug_rgb` — Publish `/processed_image_rgb` when true.

I/O:
- In: `/raspicam_node/image/compressed`
- Out: `/colour` (`std_msgs/String`: `R`, `G`, `B`, or `N`), `/processed_image_rgb`

### rgb_management
Purpose: Adjust the controller’s `~base_speed` based on color.

Params:
- `target_param` — Parameter to set (default `/error_management/base_speed`).
- `default_speed`, `red_speed`, `green_speed`, `blue_speed` — Values to write.

I/O:
- In: `/colour`
- Side effect: sets the `target_param` on the parameter server.

### line_statistics
Purpose: Aggregate per-lap stats; log CSV; publish periodic summaries and optional per-lap sample CSVs.

Params:
- `error_topic`, `cmd_topic`
- `enable_keyboard` — If true, a small non-blocking keyboard loop is enabled:
  - Spacebar — end lap and write a CSV line
  - `r` — reset counters and restart lap index at 1
- `status_period` — seconds between console updates
- `log_file` — path to CSV file; header is written if file was empty
- `sample_log_dir` — directory for detailed per-lap samples (`type,error,linear_x,angular_z,...`)

I/O:
- In: `/error`, `/cmd_vel`
- Out: `lap_statistics` (`std_msgs/String`)

CSV columns for `log_file`:
`laptime_sec,samples,mean_error,rmse,stddev,mean_abs_error,max_abs_error,avg_linear,avg_abs_linear,avg_angular,avg_abs_angular,max_linear,max_abs_angular`

### teleop
Purpose: Simple keyboard teleop publisher to `/cmd_vel` (10 Hz).

Key bindings (examples):
- Forward: `i` or `w`; Backward: `,` or `s`
- Turn in place: `j` (left), `l` (right); gentle arcs: `u`/`o`
- Stop: `k` or spacebar


## Data Logging & Plotting
- Default log file (full launch): `~/Line_lap_logs/line_laps.csv`.
- Create the directory if needed: `mkdir -p ~/Line_lap_logs` before launching.
- Optional per-lap sample CSVs when `sample_log_dir` is provided (e.g., `~/laps_stats`).

These CSVs can be analyzed with your preferred tools (e.g., Python/pandas, spreadsheets).


## Tips & Troubleshooting
- Submodule not present: run `git submodule update --init --recursive`.
- Failed to open stats CSV: ensure the directory of `stats_log_file` exists before launch.
- No camera on desktop: use `line_follower_no_motor.launch` and consider feeding a bag file or a USB camera with a compatible node.
- WiringPi errors or no GPIO: only launch `motor_control` on Raspberry Pi with WiringPi installed.
- Adjusting speed by color: ensure `rgb_management`’s `target_param` matches the controller’s private param path (default `/error_management/base_speed`).


## License
All non-submodule packages in this repository are licensed under the MIT License (declared in each package’s `package.xml`).

The `raspicam_node` submodule remains under its upstream license from UbiquityRobotics.

## Maintainers
- Rodrigo Oliveira — 1231491@isep.ipp.pt
- Bruno Moreira — 1220892@isep.ipp.pt

## Acknowledgements
- Camera driver: UbiquityRobotics `raspicam_node` (submodule).
- ROS Melodic / catkin.

