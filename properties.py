import bpy
from bpy.props import (
    StringProperty, IntProperty, FloatProperty, EnumProperty, BoolProperty
)


class DroidSlamProperties(bpy.types.PropertyGroup):

    # --- paths ---
    droid_slam_dir: StringProperty(
        name="DROID-SLAM Dir",
        description="Root directory of the DROID-SLAM repository",
        default="/mnt/share/dev-lbringer/camera_tracking/DROID-SLAM",
        subtype='DIR_PATH',
    )
    conda_env: StringProperty(
        name="Conda Env",
        description="Name of the conda environment that has DROID-SLAM installed",
        default="droidenv",
    )
    input_path: StringProperty(
        name="Input",
        description="Path to a video file (.mp4) or a directory of images",
        subtype='FILE_PATH',
    )
    output_dir: StringProperty(
        name="Output Dir",
        description="Directory where the reconstruction and trajectory will be saved",
        subtype='DIR_PATH',
    )

    # --- calibration ---
    calib_mode: EnumProperty(
        name="Calibration",
        items=[
            ('FILE',   'File',   'Point to an existing calib .txt file'),
            ('MANUAL', 'Manual', 'Enter fx, fy, cx, cy directly'),
        ],
        default='MANUAL',
    )
    calib_file: StringProperty(
        name="Calib File",
        subtype='FILE_PATH',
    )
    fx: FloatProperty(name="fx", default=1000.0, min=1.0)
    fy: FloatProperty(name="fy", default=1000.0, min=1.0)
    cx: FloatProperty(name="cx", default=960.0,  min=0.0)
    cy: FloatProperty(name="cy", default=540.0,  min=0.0)

    # --- run settings ---
    stride: IntProperty(
        name="Stride",
        description="Use every Nth frame",
        default=1, min=1,
    )
    buffer: IntProperty(
        name="Buffer",
        description="Max keyframe buffer size",
        default=512, min=64,
    )

    # --- import settings ---
    start_frame: IntProperty(
        name="Start Frame",
        description="Blender frame that corresponds to frame 0 of the trajectory",
        default=1,
    )
    end_frame: IntProperty(
        name="End Frame",
        description="Last Blender frame to import (-1 = all frames)",
        default=-1,
        min=-1,
    )
    import_pointcloud: BoolProperty(
        name="Import Point Cloud",
        description="Also import the reconstructed point cloud as a mesh",
        default=True,
    )
    point_radius: FloatProperty(
        name="Point Radius",
        description="Display radius of each point in the Blender viewport (world units)",
        default=0.005,
        min=0.0001,
        soft_max=0.1,
        precision=4,
    )

    # --- runtime state ---
    status: StringProperty(default="Ready")
    last_tum_path: StringProperty(default="")
    last_ply_path: StringProperty(default="")
    last_log_path: StringProperty(default="")
