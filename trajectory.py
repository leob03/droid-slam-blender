"""
Import a DROID-SLAM TUM trajectory into Blender as an animated camera.

Coordinate system fix (same approach as vipe_colmap2blender.py):
  - Per-pose: M = diag(1, -1, -1) applied as right-multiply on the
    camera-to-world rotation, converting from DROID/OpenCV camera axes
    (X right, Y down, Z forward) to Blender camera axes (X right, Y up, Z back).
  - Global: parent Empty with -90 deg X rotation, converting from
    DROID world space (Z forward, Y down) to Blender world space (Z up).
"""

import math
from mathutils import Matrix, Vector, Quaternion


# DROID camera axes → Blender camera axes
M = Matrix(((1, 0, 0),
            (0, -1, 0),
            (0, 0, -1)))


def import_tum_trajectory(tum_path: str, start_frame: int = 1,
                          calib_path: str = "", fx: float = 0, fy: float = 0,
                          cx: float = 0, cy: float = 0) -> None:
    import bpy

    cam_name = "DROID-SLAM_Camera"
    if cam_name in bpy.data.objects:
        cam_obj = bpy.data.objects[cam_name]
        cam_data = cam_obj.data
    else:
        cam_data = bpy.data.cameras.new(name=cam_name)
        cam_obj = bpy.data.objects.new(cam_name, cam_data)
        bpy.context.collection.objects.link(cam_obj)

    # --- apply intrinsics ---
    # Read from calib file if provided, otherwise use passed values
    if calib_path:
        import numpy as np
        calib = np.loadtxt(calib_path, delimiter=" ")
        fx, fy, cx, cy = calib[:4]

    if fx > 0:
        # Pixel-units FOV trick: set sensor_width = image_width (px) and
        # lens = fx (px). Preserves FOV exactly without knowing real sensor size.
        W = round(cx * 2)
        H = round(cy * 2)
        cam_data.type = 'PERSP'
        cam_data.sensor_fit = 'HORIZONTAL'
        cam_data.sensor_width  = float(W)
        cam_data.sensor_height = float(H)
        cam_data.lens = float(fx)
        bpy.context.scene.render.resolution_x = W
        bpy.context.scene.render.resolution_y = H

    bpy.context.scene.camera = cam_obj
    cam_obj.rotation_mode = "QUATERNION"

    # clear existing animation
    cam_obj.animation_data_clear()

    with open(tum_path) as f:
        for frame_offset, line in enumerate(f):
            line = line.strip()
            if not line:
                continue

            vals = [float(v) for v in line.split()]
            if len(vals) == 8:
                _, tx, ty, tz, qx, qy, qz, qw = vals
            elif len(vals) == 7:
                tx, ty, tz, qx, qy, qz, qw = vals
            else:
                continue

            R_c2w = Quaternion((qw, qx, qy, qz)).to_matrix()
            t_c2w = Vector((tx, ty, tz))

            mat = (R_c2w @ M).to_4x4()
            mat.translation = t_c2w

            cam_obj.matrix_world = mat

            frame_num = start_frame + frame_offset
            cam_obj.keyframe_insert(data_path="location",              frame=frame_num)
            cam_obj.keyframe_insert(data_path="rotation_quaternion",   frame=frame_num)

    # global world-coordinate fix via parent Empty
    empty_name = "DROID-SLAM_GlobalFix"
    if empty_name in bpy.data.objects:
        empty = bpy.data.objects[empty_name]
    else:
        empty = bpy.data.objects.new(empty_name, None)
        bpy.context.collection.objects.link(empty)

    empty.rotation_mode = "XYZ"
    empty.rotation_euler = (math.radians(-90.0), 0.0, 0.0)
    cam_obj.parent = empty

    bpy.context.scene.frame_start = start_frame
    bpy.context.scene.frame_end   = start_frame + frame_offset

    print(f"✔ Imported {frame_offset + 1} poses from {tum_path}")


def import_pointcloud_ply(ply_path: str) -> None:
    import bpy

    # Blender 4.x uses wm.ply_import; older versions use import_mesh.ply
    try:
        bpy.ops.wm.ply_import(filepath=ply_path)
    except AttributeError:
        bpy.ops.import_mesh.ply(filepath=ply_path)

    # parent to the same global fix empty so it lives in the same space
    empty_name = "DROID-SLAM_GlobalFix"
    if empty_name in bpy.data.objects:
        obj = bpy.context.active_object
        if obj:
            obj.parent = bpy.data.objects[empty_name]

    print(f"✔ Imported point cloud from {ply_path}")
