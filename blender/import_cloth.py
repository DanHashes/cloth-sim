"""cloth-sim Blender import script.

Run from Blender's Scripting tab (Text Editor -> Open -> this file -> Run
Script), or via command line:

    blender --python blender/import_cloth.py

It builds a flat grid mesh with the same vertex count and ordering as the
C++ simulator's ClothMesh, adds a Mesh Cache modifier pointing at an
exported .pc2 file, and sets the scene's playback range to match. The
Mesh Cache modifier overwrites vertex positions every frame directly from
the .pc2 data, so this script's only real job is to get the vertex COUNT,
ORDER, and FACE topology to match -- the initial flat position barely
matters since it is replaced as soon as playback starts.

Edit the settings below to match the scene you exported with the CLI
(see scenes/*.txt -- resx/resy/width/height/frames must match, and
PC2_PATH must point at the .pc2 the CLI wrote).
"""

import os

import bpy

# --- Edit these to match the scene you simulated -----------------------
RES_X = 20
RES_Y = 20
WIDTH = 2.0
HEIGHT = 2.0
FRAME_COUNT = 300
# Path to the .pc2 file, relative to this .blend file ("//" prefix) or
# absolute. Defaults to a file living alongside the repo's scenes/ output.
PC2_PATH = "//cloth_output.pc2"
OBJECT_NAME = "ClothSim"
# -------------------------------------------------------------------------


def build_cloth_mesh(name, res_x, res_y, width, height):
    """Builds a flat grid with the same vertex index scheme as ClothMesh:
    index = y * res_x + x (see ClothMesh::particleIndex in the C++ source).
    Getting this order right is what matters -- the Mesh Cache modifier
    will move every vertex to its .pc2 position on playback regardless of
    where we put it here.
    """
    verts = []
    for y in range(res_y):
        for x in range(res_x):
            verts.append((
                x * (width / (res_x - 1)),
                0.0,
                y * (height / (res_y - 1)),
            ))

    faces = []
    for y in range(res_y - 1):
        for x in range(res_x - 1):
            top_left = y * res_x + x
            top_right = y * res_x + (x + 1)
            bottom_left = (y + 1) * res_x + x
            bottom_right = (y + 1) * res_x + (x + 1)
            faces.append((top_left, bottom_left, bottom_right, top_right))

    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    # Remove any stale object left over from a previous run so re-running
    # this script doesn't pile up duplicates.
    existing = bpy.data.objects.get(name)
    if existing is not None:
        bpy.data.objects.remove(existing, do_unlink=True)

    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def add_mesh_cache_modifier(obj, pc2_path, frame_count):
    modifier = obj.modifiers.new(name="ClothCache", type='MESH_CACHE')
    modifier.cache_format = 'PC2'
    modifier.filepath = pc2_path
    # These two settings tell Blender how to remap the cache's raw XYZ
    # floats onto the mesh's local axes -- they are not free-form labels.
    # Verified empirically: comparing Blender's evaluated mesh against the
    # raw .pc2 bytes vertex-for-vertex, forward_axis=POS_Y / up_axis=POS_Z
    # is the only combination (of all 24 valid axis pairs) that produces an
    # exact passthrough for cloth-sim's data, which stores gravity along -Y
    # rather than Blender's own -Z-down convention.
    modifier.forward_axis = 'POS_Y'
    modifier.up_axis = 'POS_Z'
    modifier.play_mode = 'SCENE'
    modifier.frame_start = 0
    modifier.frame_scale = 1.0
    return modifier


def main():
    obj = build_cloth_mesh(OBJECT_NAME, RES_X, RES_Y, WIDTH, HEIGHT)
    add_mesh_cache_modifier(obj, PC2_PATH, FRAME_COUNT)

    scene = bpy.context.scene
    scene.frame_start = 0
    scene.frame_end = max(0, FRAME_COUNT - 1)
    scene.frame_set(0)

    resolved_path = bpy.path.abspath(PC2_PATH)
    exists = os.path.isfile(resolved_path)
    print(
        f"[cloth-sim] created '{obj.name}' with {RES_X * RES_Y} vertices, "
        f"{len(obj.data.polygons)} faces; Mesh Cache -> {resolved_path} "
        f"(exists: {exists})"
    )


if __name__ == "__main__":
    main()
