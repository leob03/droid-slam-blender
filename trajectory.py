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


def import_pointcloud_ply(ply_path: str, point_radius: float = 0.005) -> None:
    import bpy

    obj = _import_ply_colored(ply_path, point_radius=point_radius)

    empty_name = "DROID-SLAM_GlobalFix"
    if empty_name in bpy.data.objects and obj:
        obj.parent = bpy.data.objects[empty_name]

    print(f"✔ Imported point cloud from {ply_path}")


def _parse_ply(ply_path: str):
    """Parse a binary PLY file, returning (coords, colors) as float32 arrays."""
    import numpy as np

    try:
        from plyfile import PlyData
        plydata = PlyData.read(ply_path)
        verts = plydata["vertex"].data
        coords = np.stack([verts["x"], verts["y"], verts["z"]], axis=1).astype(np.float32)
        if {"red", "green", "blue"}.issubset(verts.dtype.names):
            colors = np.stack([verts["red"], verts["green"], verts["blue"]], axis=1).astype(np.float32) / 255.0
        else:
            colors = np.ones((len(coords), 3), dtype=np.float32)
        return coords, colors
    except ImportError:
        pass

    # Manual fallback parser
    with open(ply_path, 'rb') as f:
        header_bytes = b""
        while True:
            line = f.readline()
            header_bytes += line
            if line.strip() == b"end_header":
                break

        header = header_bytes.decode('utf-8')
        n_verts = int(next(l for l in header.splitlines() if l.startswith("element vertex")).split()[-1])

        type_map = {
            'double': np.float64, 'float': np.float32,
            'uchar': np.uint8,    'uint8': np.uint8,
            'int': np.int32,      'uint': np.uint32,
        }
        props = []
        for l in header.splitlines():
            if l.startswith("property"):
                parts = l.split()
                props.append((parts[2], type_map.get(parts[1], np.float32)))

        dtype = np.dtype(props)
        data  = np.frombuffer(f.read(n_verts * dtype.itemsize), dtype=dtype)

    coords = np.stack([data['x'], data['y'], data['z']], axis=1).astype(np.float32)
    if all(c in data.dtype.names for c in ('red', 'green', 'blue')):
        colors = np.stack([data['red'], data['green'], data['blue']], axis=1).astype(np.float32) / 255.0
    else:
        colors = np.ones((len(coords), 3), dtype=np.float32)

    return coords, colors


def _import_ply_colored(ply_path: str, point_radius: float = 0.005):
    import bpy
    import numpy as np

    coords, colors = _parse_ply(ply_path)
    n_verts = len(coords)

    # --- build mesh ---
    mesh = bpy.data.meshes.new("DROID-SLAM_PointCloud")
    mesh.vertices.add(n_verts)
    mesh.vertices.foreach_set("co", coords.flatten())
    mesh.update()

    # RGBA color attribute — same approach as DA3-blender
    attr = mesh.attributes.new(name="point_color", type='FLOAT_COLOR', domain='POINT')
    rgba = np.concatenate([colors, np.ones((n_verts, 1), dtype=np.float32)], axis=1)
    attr.data.foreach_set("color", rgba.flatten())

    # --- create object ---
    obj = bpy.data.objects.new("DROID-SLAM_PointCloud", mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj

    # --- material: Emission shader so colors show without scene lighting ---
    mat = bpy.data.materials.new(name="DROID-SLAM_PointCloud_Mat")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()

    attr_node = nt.nodes.new("ShaderNodeAttribute")
    attr_node.attribute_name = "point_color"
    attr_node.location = (-300, 0)

    emission = nt.nodes.new("ShaderNodeEmission")
    emission.location = (0, 0)

    out = nt.nodes.new("ShaderNodeOutputMaterial")
    out.location = (250, 0)

    nt.links.new(attr_node.outputs["Color"], emission.inputs["Color"])
    nt.links.new(emission.outputs["Emission"], out.inputs["Surface"])

    mesh.materials.append(mat)

    # --- Geometry Nodes: MeshToPoints + SetMaterial ---
    # Without this a vertex-only mesh won't render the material at all.
    _add_point_geo_nodes(obj, mat, point_radius=point_radius)

    return obj


def _add_point_geo_nodes(obj, mat, point_radius: float = 0.005):
    import bpy

    geo_mod = obj.modifiers.new(name="GeometryNodes", type='NODES')
    ng = bpy.data.node_groups.new(name="PointCloudViz", type='GeometryNodeTree')
    geo_mod.node_group = ng

    ng.interface.new_socket(name="Geometry", in_out="INPUT",  socket_type="NodeSocketGeometry")
    radius_socket = ng.interface.new_socket(name="Radius",   in_out="INPUT",  socket_type="NodeSocketFloat")
    radius_socket.default_value = point_radius
    radius_socket.min_value     = 0.0001
    ng.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")

    n_in  = ng.nodes.new('NodeGroupInput')
    n_out = ng.nodes.new('NodeGroupOutput')

    m2p = ng.nodes.new('GeometryNodeMeshToPoints')

    set_mat = ng.nodes.new('GeometryNodeSetMaterial')
    set_mat.inputs[2].default_value = mat  # index 2 = Material socket

    ng.links.new(n_in.outputs['Geometry'], m2p.inputs['Mesh'])
    ng.links.new(n_in.outputs['Radius'],   m2p.inputs['Radius'])
    ng.links.new(m2p.outputs['Points'],       set_mat.inputs['Geometry'])
    ng.links.new(set_mat.outputs['Geometry'], n_out.inputs['Geometry'])

    # set the modifier's Radius input to match the import setting
    for item in ng.interface.items_tree:
        if item.name == "Radius":
            geo_mod[item.identifier] = float(point_radius)
            break
