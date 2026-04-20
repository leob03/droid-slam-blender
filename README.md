# DROID-SLAM in Blender

A Blender add-on that runs [DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM) camera tracking directly from Blender and imports the resulting trajectory and point cloud into the scene.

https://github.com/user-attachments/assets/613018a9-e775-4e56-9653-b804a5caf607

## Installation

### 1. Set up DROID-SLAM

Clone the repo:
```bash
git clone --recursive https://github.com/princeton-vl/DROID-SLAM.git
cd DROID-SLAM
```

Download the model weights:
```bash
./tools/download_model.sh
```

> **Note:** The upstream DROID-SLAM repo uses a `.venv` for installation, but this addon launches DROID-SLAM via `conda activate`. You need to set up a **conda environment** instead.

Create and activate a conda environment (named `droidenv` to match the addon default):
```bash
conda create -n droidenv python=3.9
conda activate droidenv
```

Install dependencies:
```bash
pip install -r requirements.txt
pip install thirdparty/lietorch
pip install thirdparty/pytorch_scatter
pip install -e .
```

Verify CUDA is available:
```bash
python -c "import torch; print(torch.version.cuda)"
```

### 2. Install the Blender addon

- Zip the `droid_slam_blender/` folder
- In Blender: **Edit → Preferences → Add-ons → Install** → select the zip
- Enable **Camera: DROID-SLAM**

Or symlink directly into Blender's addons directory:
```bash
ln -s /path/to/droid_slam_blender \
      ~/.config/blender/<version>/scripts/addons/droid_slam_blender
```

## Usage

Open the **N-panel** in the 3D Viewport (press `N`) → **DROID-SLAM** tab.

### Paths
| Field | Description |
|---|---|
| DROID-SLAM Dir | Root of the DROID-SLAM repo (must contain `droid.pth`) |
| Conda Env | Name of the conda env (default: `droidenv`) |
| Input | Video file (`.mp4`) or directory of images |
| Output Dir | Where `.pth` and `.ply` files are saved |

### Calibration
- **File** — point to an existing `calib/*.txt`
- **Manual** — enter `fx fy cx cy` directly (written to a temp file automatically)

### Settings
- **Stride** — use every Nth frame (1 = all frames)
- **Buffer** — keyframe buffer size (increase for longer sequences)
- **Start Frame / End Frame** — clip the input video to this frame range before passing to the model (`-1` = process all frames)

### Running
Click **Run DROID-SLAM**. The process runs in the background; Blender stays responsive and streams live logs to the terminal. When done, the trajectory and (optionally) the point cloud are imported automatically.

You can also manually import an existing trajectory or `.ply` using the **Trajectory** / **Point Cloud** buttons in the Import section.

## Coordinate system
Follows the same convention as `vipe_colmap2blender.py`:
- Per-pose: `M = diag(1, -1, -1)` right-multiplied onto the camera-to-world rotation (DROID/OpenCV → Blender camera axes)
- Global: parent Empty with `-90° X` rotation (DROID world space → Blender world space)
