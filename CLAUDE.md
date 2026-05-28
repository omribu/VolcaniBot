# Person-Following Robot — Project Context

A ROS 2 robot that follows a specific person selected by the operator via joystick.
Perception is provided by the cloned `yolo_ros` package; the follower logic is ours to build.

## Hardware / Platform (all verified working)
- Jetson AGX Orin (64 GB)
- JetPack 6.2.1 → L4T r36.4, CUDA 12.6, TensorRT 10.3
- Ubuntu 22.04, ROS 2 Humble
- Intel RealSense depth camera (bring-up still TODO)
- Joystick for operator control (button index TBD)

## GPU / inference status — DONE, do not redo
- PyTorch is NVIDIA's Jetson build (`torch 2.5.0a0+...nv24.08`), `torch.cuda.is_available() == True`
- torchvision `0.20.0a0+afc54f7` (NVIDIA build), imports cleanly
- ultralytics 8.4.51, runs on GPU
- Power: MAXN (`nvpmodel -m 0`, persistent) + `jetson_clocks` via systemd service (runs every boot)
- TensorRT engine built ON-DEVICE and benchmarked:
  - File: `models/yolo11m.engine` (FP16, imgsz=480x640)
  - Inference: ~5.4 ms (vs 159 ms cold FP32 .pt) — ~30x speedup
  - End-to-end ~18 ms incl. postprocess → ~55 FPS ceiling, plenty of headroom

## CRITICAL build constraints
- DO NOT use venv, `uv sync`, or bulk `pip install -r requirements.txt` in this workspace.
  Any of those pull the CPU-only PyPI torch and silently break GPU inference.
  Build `yolo_ros` against SYSTEM Python with colcon only.
- If a single python module is missing at runtime, install JUST that one with
  `pip3 install --user <module>` — never a bulk install that could touch torch.
- TensorRT engine is non-portable: tied to this exact Orin + TensorRT 10.3.
  Re-export if JetPack is reflashed or moving to another board.
- Engine input size is LOCKED to 480x640 → launch MUST pass
  `imgsz_height:=480 imgsz_width:=640` to match, or it errors / mis-scales.

## Perception layer: mgonzs13/yolo_ros (cloned in this repo)
Relevant topics:
- `/yolo/tracking` — detections WITH persistent tracking `id` (2D bbox + class + id)
- `/yolo/detections_3d` — same detections + 3D bbox from depth (carries `id` AND 3D centroid)
- `/yolo/debug_image` — rviz2 visualization
Relevant service:
- `/yolo/enable` (std_srvs/SetBool) — turn detector on/off

Detection msg fields we use (yolo_msgs/Detection):
- `id` (tracking id), `class_name` (filter == "person"), `bbox` (2D),
  `bbox3d.center.position` (x/y/z in target_frame)

Planned launch (run with the TensorRT engine):
```
ros2 launch yolo_bringup yolo.launch.py \
  use_3d:=True use_tracking:=True \
  model:=<abs path>/models/yolo11m.engine \
  tracker:=botsort.yaml device:=cuda:0 half:=True \
  imgsz_height:=480 imgsz_width:=640 \
  input_image_topic:=/camera/color/image_raw \
  input_depth_topic:=/camera/aligned_depth_to_color/image_raw \
  input_depth_info_topic:=/camera/aligned_depth_to_color/camera_info \
  depth_image_units_divisor:=1000 target_frame:=base_link
```
Use `tracker:=botsort.yaml` (with ReID) for robust IDs, not the default bytetrack.

## Goal application — the follower node WE build (not in yolo_ros)
A single ROS 2 Python node:
1. Subscribe to `/joy`. On target button press, TOGGLE follow state.
   - Turning ON: call `/yolo/enable` true.
   - Turning OFF: call `/yolo/enable` false, clear latched target, publish zero `/cmd_vel`.
2. On enable, subscribe to `/yolo/detections_3d`. Pick the NEAREST person
   (class_name == "person", smallest `bbox3d.center.position` distance, e.g. z).
   Latch that detection's `id`.
3. Each frame, find the detection whose `id == latched_id`, take its 3D centroid,
   run a controller (heading + standoff distance) → publish `/cmd_vel`.
4. If latched `id` absent for N consecutive frames → publish zero velocity (STOP),
   keep waiting. Resume when it reappears.

## Known design risk to handle in the follower
Re-acquisition by `id` alone is fragile: when a person FULLY exits the frame the
tracker usually assigns a NEW id on return (BoT-SORT+ReID only *probably* recovers it).
So "stop until person reappears" by id-match may never re-trigger.
Mitigation options (decide during implementation):
- After lost > T seconds, re-acquire by "nearest person" instead of id, or
- Require an explicit joystick re-press to re-lock.

## Next steps
- [ ] RealSense bring-up (build librealsense + realsense-ros for JetPack 6,
      confirm `aligned_depth_to_color` topic publishes)
- [ ] colcon build yolo_ros against system python
- [ ] smoke test: confirm `/yolo/tracking` publishes from a test RGB source
- [ ] write the follower node
- [ ] tune controller gains + standoff distance + lost-track timeout
