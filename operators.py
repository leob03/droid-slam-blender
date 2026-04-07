import bpy
import os
import subprocess
import tempfile

# module-level handle for the running subprocess
_process  = None
_timer    = None
_log_file = None  # open file handle for the log


def _poll_process():
    """Timer callback that checks whether the DROID-SLAM subprocess has finished."""
    global _process, _timer, _log_file

    if _process is None:
        return None  # cancel timer

    retcode = _process.poll()
    if retcode is None:
        return 1.0  # still running – check again in 1 s

    if _log_file:
        _log_file.close()
        _log_file = None

    props = bpy.context.scene.droid_slam

    if retcode == 0:
        props.status = "Done"
        if props.last_tum_path and os.path.exists(props.last_tum_path):
            from .trajectory import import_tum_trajectory
            import_tum_trajectory(props.last_tum_path, props.start_frame)

        if props.import_pointcloud and props.last_ply_path and os.path.exists(props.last_ply_path):
            from .trajectory import import_pointcloud_ply
            import_pointcloud_ply(props.last_ply_path)
    else:
        props.status = f"Error (exit {retcode}) — see log"

    _process = None
    _timer   = None
    return None  # cancel timer


class DROIDSLAM_OT_Run(bpy.types.Operator):
    bl_idname  = "droid_slam.run"
    bl_label   = "Run DROID-SLAM"
    bl_description = "Launch DROID-SLAM in the background and import the result when done"

    def execute(self, context):
        global _process, _timer, _log_file
        props = context.scene.droid_slam

        if _process is not None and _process.poll() is None:
            self.report({'WARNING'}, "DROID-SLAM is already running.")
            return {'CANCELLED'}

        # --- validate ---
        if not props.input_path:
            self.report({'ERROR'}, "No input path specified.")
            return {'CANCELLED'}

        droid_dir  = bpy.path.abspath(props.droid_slam_dir)
        input_path = bpy.path.abspath(props.input_path)
        output_dir = bpy.path.abspath(props.output_dir) if props.output_dir else droid_dir
        os.makedirs(output_dir, exist_ok=True)

        # --- calibration file ---
        if props.calib_mode == 'FILE':
            calib_path = bpy.path.abspath(props.calib_file)
        else:
            calib_fd, calib_path = tempfile.mkstemp(suffix=".txt", prefix="droid_calib_")
            with os.fdopen(calib_fd, 'w') as f:
                f.write(f"{props.fx} {props.fy} {props.cx} {props.cy}\n")

        # --- output paths ---
        basename   = os.path.splitext(os.path.basename(input_path))[0]
        recon_path = os.path.join(output_dir, f"{basename}.pth")
        tum_path   = os.path.join(droid_dir, "traj_full_tum.txt")
        ply_path   = os.path.join(output_dir, f"{basename}.ply")
        log_path   = os.path.join(output_dir, f"{basename}_droid.log")

        props.last_tum_path = tum_path
        props.last_ply_path = ply_path
        props.last_log_path = log_path

        # --- resolve conda base ---
        try:
            conda_base = subprocess.check_output(
                ["conda", "info", "--base"], text=True
            ).strip()
        except Exception as e:
            self.report({'ERROR'}, f"Could not locate conda: {e}")
            return {'CANCELLED'}

        conda_sh = os.path.join(conda_base, "etc", "profile.d", "conda.sh")

        # Build the demo.py args as a single string for the bash -c call
        demo_args = " ".join([
            f'"{os.path.join(droid_dir, "demo.py")}"',
            "--imagedir",            f'"{input_path}"',
            "--calib",               f'"{calib_path}"',
            "--weights",             f'"{os.path.join(droid_dir, "droid.pth")}"',
            "--stride",              str(props.stride),
            "--buffer",              str(props.buffer),
            "--disable_vis",
            "--reconstruction_path", f'"{recon_path}"',
        ])

        # Use bash -c to source conda and activate the env — exactly what
        # a terminal does. Unset LD_LIBRARY_PATH first so Blender's copy
        # doesn't shadow CUDA/cuDNN libraries.
        # DA3-blender installs its own cuDNN into Blender's LD_LIBRARY_PATH,
        # which conflicts with PyTorch's bundled cuDNN. Fix: activate the conda
        # env first, then prepend PyTorch's own lib dir so its cuDNN wins.
        view_recon = os.path.join(droid_dir, "view_reconstruction.py")

        bash_cmd = (
            f"source \"{conda_sh}\" && "
            f"conda activate \"{props.conda_env}\" && "
            f"TORCH_LIB=$(python -c \"import torch, os; print(os.path.join(os.path.dirname(torch.__file__), 'lib'))\") && "
            f"export LD_LIBRARY_PATH=\"$TORCH_LIB\" && "
            f"python {demo_args} && "
            f"python \"{view_recon}\" \"{recon_path}\""
        )
        cmd = ["bash", "-c", bash_cmd]

        # strip CUDA and library path vars that Blender may have set
        env = os.environ.copy()
        for var in ('LD_LIBRARY_PATH', 'CUDA_VISIBLE_DEVICES', 'CUDA_HOME',
                    'CUDA_ROOT', 'CUDA_PATH'):
            env.pop(var, None)

        _log_file = open(log_path, 'w')
        _log_file.write("Command: " + " ".join(cmd) + "\n\n")
        _log_file.flush()

        _process = subprocess.Popen(
            cmd,
            cwd=droid_dir,
            stdout=_log_file,
            stderr=subprocess.STDOUT,
            env=env,
        )

        props.status = "Running…"
        _timer = bpy.app.timers.register(_poll_process, first_interval=2.0)

        self.report({'INFO'}, f"DROID-SLAM started. Log: {log_path}")
        return {'FINISHED'}


class DROIDSLAM_OT_Cancel(bpy.types.Operator):
    bl_idname  = "droid_slam.cancel"
    bl_label   = "Cancel"
    bl_description = "Kill the running DROID-SLAM process"

    def execute(self, context):
        global _process, _log_file
        if _process is not None and _process.poll() is None:
            _process.terminate()
            context.scene.droid_slam.status = "Cancelled"
        if _log_file:
            _log_file.close()
            _log_file = None
        return {'FINISHED'}


class DROIDSLAM_OT_OpenLog(bpy.types.Operator):
    bl_idname  = "droid_slam.open_log"
    bl_label   = "Open Log"
    bl_description = "Open the log file in a text editor"

    def execute(self, context):
        log_path = context.scene.droid_slam.last_log_path
        if not log_path or not os.path.exists(log_path):
            self.report({'WARNING'}, "No log file found.")
            return {'CANCELLED'}

        # load into Blender's text editor
        text_name = os.path.basename(log_path)
        if text_name in bpy.data.texts:
            bpy.data.texts.remove(bpy.data.texts[text_name])
        with open(log_path) as f:
            content = f.read()
        text = bpy.data.texts.new(text_name)
        text.write(content)

        # open a text editor area if possible
        for area in context.screen.areas:
            if area.type == 'TEXT_EDITOR':
                area.spaces.active.text = text
                break

        self.report({'INFO'}, f"Log loaded: {text_name}")
        return {'FINISHED'}


class DROIDSLAM_OT_ImportTrajectory(bpy.types.Operator):
    bl_idname  = "droid_slam.import_trajectory"
    bl_label   = "Import Trajectory"
    bl_description = "Import traj_full_tum.txt as an animated Blender camera"

    filepath: bpy.props.StringProperty(subtype='FILE_PATH')

    def invoke(self, context, event):
        props = context.scene.droid_slam
        if props.last_tum_path:
            self.filepath = props.last_tum_path
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        from .trajectory import import_tum_trajectory
        import_tum_trajectory(self.filepath, context.scene.droid_slam.start_frame)
        return {'FINISHED'}


class DROIDSLAM_OT_ImportPointCloud(bpy.types.Operator):
    bl_idname  = "droid_slam.import_pointcloud"
    bl_label   = "Import Point Cloud"
    bl_description = "Import a DROID-SLAM .ply reconstruction"

    filepath: bpy.props.StringProperty(subtype='FILE_PATH')

    def invoke(self, context, event):
        props = context.scene.droid_slam
        if props.last_ply_path:
            self.filepath = props.last_ply_path
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        from .trajectory import import_pointcloud_ply
        import_pointcloud_ply(self.filepath)
        return {'FINISHED'}


CLASSES = [
    DROIDSLAM_OT_Run,
    DROIDSLAM_OT_Cancel,
    DROIDSLAM_OT_OpenLog,
    DROIDSLAM_OT_ImportTrajectory,
    DROIDSLAM_OT_ImportPointCloud,
]
