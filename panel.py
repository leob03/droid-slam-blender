import bpy
from . import operators as ops


class DROIDSLAM_PT_Panel(bpy.types.Panel):
    bl_label       = "DROID-SLAM"
    bl_idname      = "DROIDSLAM_PT_Panel"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = "DROID-SLAM"

    def draw(self, context):
        layout = self.layout
        props  = context.scene.droid_slam

        # --- Paths ---
        box = layout.box()
        box.label(text="Paths", icon='FILE_FOLDER')
        box.prop(props, "droid_slam_dir")
        box.prop(props, "conda_env")
        box.prop(props, "input_path")
        box.prop(props, "output_dir")

        # --- Calibration ---
        box = layout.box()
        box.label(text="Calibration", icon='CAMERA_DATA')
        box.prop(props, "calib_mode", expand=True)
        if props.calib_mode == 'FILE':
            box.prop(props, "calib_file")
        else:
            row = box.row(align=True)
            row.prop(props, "fx")
            row.prop(props, "fy")
            row = box.row(align=True)
            row.prop(props, "cx")
            row.prop(props, "cy")

        # --- Run settings ---
        box = layout.box()
        box.label(text="Settings", icon='SETTINGS')
        row = box.row(align=True)
        row.prop(props, "stride")
        row.prop(props, "buffer")

        # --- Run / Cancel ---
        box = layout.box()
        is_running = props.status == "Running…"
        row = box.row(align=True)
        row.operator("droid_slam.run",    icon='PLAY', text="Run DROID-SLAM")
        row.operator("droid_slam.cancel", icon='X',    text="")
        status_icon = 'TIME' if is_running else ('CHECKMARK' if props.status == "Done" else 'INFO')
        box.label(text=f"Status: {props.status}", icon=status_icon)
        if props.last_log_path:
            box.operator("droid_slam.open_log", icon='TEXT', text="Open Log in Text Editor")

        # --- Import ---
        box = layout.box()
        box.label(text="Import", icon='IMPORT')
        box.prop(props, "start_frame")
        box.prop(props, "import_pointcloud")
        row = box.row(align=True)
        row.operator("droid_slam.import_trajectory", icon='ARMATURE_DATA',   text="Trajectory")
        row.operator("droid_slam.import_pointcloud", icon='POINTCLOUD_DATA', text="Point Cloud")


CLASSES = [DROIDSLAM_PT_Panel]
