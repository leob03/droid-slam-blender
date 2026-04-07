bl_info = {
    "name":        "DROID-SLAM",
    "author":      "lbringer",
    "version":     (0, 1, 0),
    "blender":     (3, 0, 0),
    "location":    "3D View > N-panel > DROID-SLAM",
    "description": "Run DROID-SLAM camera tracking from within Blender and import the result",
    "category":    "Camera",
}

import bpy
from . import properties, operators, panel


def register():
    bpy.utils.register_class(properties.DroidSlamProperties)
    bpy.types.Scene.droid_slam = bpy.props.PointerProperty(type=properties.DroidSlamProperties)

    for cls in operators.CLASSES:
        bpy.utils.register_class(cls)
    for cls in panel.CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(panel.CLASSES):
        bpy.utils.unregister_class(cls)
    for cls in reversed(operators.CLASSES):
        bpy.utils.unregister_class(cls)

    del bpy.types.Scene.droid_slam
    bpy.utils.unregister_class(properties.DroidSlamProperties)
