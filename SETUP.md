# VolcaniBot — Setup & Installation

## 1. Clone (with submodules!)

This repo uses a git submodule for `yolo_ros`. A plain `git clone` leaves
`src/yolo_ros` empty and the build will fail.

    git clone --recurse-submodules <YOUR_REPO_URL>
    cd VolcaniBot

If you already cloned without `--recurse-submodules`:

    git submodule update --init --recursive

## 2. Get the model files

(Models live in `models/`. If using Git LFS, run `git lfs pull`.)
- `models/yolov8n.pt`   — portable YOLO weights.
- `models/yolo11m.engine` — TensorRT engine, GPU/CUDA/TensorRT-specific.
  May not load on a different machine; rebuild from the .pt if needed.

## 3. Install ROS dependencies

    rosdep install --from-paths src --ignore-src -r -y

## 4. Install Python dependencies

    pip install -r requirements.txt

Then install a CUDA-matched PyTorch for your machine (see PyTorch / Jetson
docs for the right wheel). Do NOT pip-install opencv-python — ROS provides it.

## 5. Build & source

    colcon build --symlink-install
    source install/setup.bash
