# droid_slam_blender

A Blender add-on that runs [DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM) camera tracking directly from Blender and imports the resulting trajectory and point cloud into the scene.

<video src="assets/demo_DROID-SLAM_blender.mp4" controls width="100%"></video>

## Installation

1. Zip the `droid_slam_blender/` folder
2. In Blender: **Edit → Preferences → Add-ons → Install** → select the zip
3. Enable **Camera: DROID-SLAM**

Or symlink the folder into Blender's addons directory:
```bash
ln -s /mnt/share/dev-lbringer/camera_tracking/droid_slam_blender \
      ~/.config/blender/<version>/scripts/addons/droid_slam_blender
```

## Usage

Open the **N-panel** in the 3D Viewport (press `N`) → **DROID-SLAM** tab.

### Paths
| Field | Description |
|---|---|
| DROID-SLAM Dir | Root of the DROID-SLAM repo (must contain `droid.pth`) |
| Conda Env | Name of the conda env (`droidenv`) |
| Input | Video file (`.mp4`) or directory of images |
| Output Dir | Where `.pth` and `.ply` files are saved |

### Calibration
- **File** — point to an existing `calib/*.txt`
- **Manual** — enter `fx fy cx cy` directly (written to a temp file automatically)

### Settings
- **Stride** — use every Nth frame (1 = all frames)
- **Buffer** — keyframe buffer size (increase for longer sequences)

### Running
Click **Run DROID-SLAM**. The process runs in the background via `conda run`; Blender stays responsive. When done, the trajectory and (optionally) the point cloud are imported automatically.

You can also manually import an existing trajectory or `.ply` using the **Trajectory** / **Point Cloud** buttons.

## Coordinate system
Follows the same convention as `vipe_colmap2blender.py`:
- Per-pose: `M = diag(1, -1, -1)` right-multiplied onto the camera-to-world rotation (DROID/OpenCV → Blender camera axes)
- Global: parent Empty with `-90° X` rotation (DROID world space → Blender world space)
