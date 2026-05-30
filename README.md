# VolcaniBot — Autonomous Agricultural Robot

ROS 2 · NVIDIA Jetson · Intel RealSense D435I

VolcaniBot is a differential-drive agricultural robot designed for field operations. It supports joystick teleoperation, real-time person detection and following using a depth camera and YOLO, and is being extended with GPS-based autonomous navigation.

---

## Key Features

- **Differential drive** via ros2_control with Roboteq serial motor driver
- **Joystick teleoperation** with twist_mux priority arbitration (joystick overrides navigation commands)
- **Person detection & tracking** — YOLO11 + BotSORT tracker running on GPU
- **Person following** — press LB to lock onto the closest detected person; the robot drives toward them proportionally based on 3D distance and bearing
- **3D position estimation** — Intel RealSense D435I aligned depth + color for metric person position in `base_link` frame
- **GPS waypoint navigation** *(planned)*

---

## Package Overview

| Package | Role |
|---|---|
| `volcanibot_bringup` | Top-level launch file (`real_robot.launch.py`) — assembles the full system |
| `volcanibot_description` | URDF/xacro robot model, meshes, and RViz configs |
| `volcanibot_diff_controller` | ros2_control hardware interface (Roboteq), joystick teleop launch and YAML configs |
| `volcanibot_follow_person` | Joystick-triggered person tracking node — locks onto closest person and publishes velocity to `/nav_vel` |
| `yolo_ros` | YOLO11 inference, BotSORT tracking, and 3D bounding-box estimation (git submodule) |

---

## Hardware

- NVIDIA Jetson (Orin / AGX)
- Intel RealSense D435I (rear-mounted, depth + RGB)
- Roboteq motor driver (serial, `/dev/ttyACM0`)
- Logitech Dual Action gamepad

---

## Quick Start

```bash
# Build
cd ~/workspaces/VolcaniBot
colcon build
source install/setup.bash

# Launch the full system
ros2 launch volcanibot_bringup real_robot.launch.py
```

> **Note:** The Roboteq motor driver (`controller`) is commented out in the launch by default. Uncomment it in `real_robot.launch.py` when the hardware is connected.

---

## Joystick Controls

| Input | Action |
|---|---|
| Button 4 (LB) | Toggle person-following on / off |
| Button 5 (RB) — hold | Enable manual drive (deadman switch) |
| Left stick Y (axis 1) | Forward / backward |
| Right stick X (axis 3) | Turn left / right |

---

## Person Following

When following is enabled (LB pressed):

1. YOLO detection activates and scans for people.
2. The robot locks onto the **closest** detected person by 3D distance.
3. It drives **backward** toward the person (camera faces rear) proportionally:
   - Starts moving when person is farther than **1.35 m**
   - Reaches maximum speed (**−0.3 m/s**) at **2.5 m**
4. Yaw corrects the bearing so the person stays directly behind.
5. Velocity drops to zero immediately if the person is lost or LB is pressed again.

Velocity commands are published on `/nav_vel` and merged by `twist_mux` (priority 50, below joystick priority 99).

---

## Topics of Interest

| Topic | Type | Description |
|---|---|---|
| `/joy` | `sensor_msgs/Joy` | Raw joystick input |
| `/joy_vel` | `geometry_msgs/Twist` | Joystick velocity (gated by deadman) |
| `/nav_vel` | `geometry_msgs/Twist` | Person-following velocity commands |
| `/volcanibot_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` | Final velocity to motor controller |
| `/yolo/detections` | `yolo_msgs/DetectionArray` | 2D YOLO detections |
| `/yolo/detections_3d` | `yolo_msgs/DetectionArray` | 3D detections in `base_link` frame |
| `/follow_person/state` | `std_msgs/String` | `DISABLED` / `SEARCHING` / `TRACKING` |
| `/follow_person/target_position` | `geometry_msgs/PointStamped` | Tracked person's 3D position |
