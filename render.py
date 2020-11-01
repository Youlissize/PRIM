import os
import bpy

# put the location to the folder where the objs are located here in this fashion
# this line will only work on windows ie C:\objects
path_to_obj_dir = os.path.join('D:\\PRIM\\PRIM\\Output', '')

# get list of all files in directory
file_list = sorted(os.listdir(path_to_obj_dir))

# get a list of files ending in 'obj'
obj_list = [item for item in file_list if item.endswith('.obj')]

# loop through the strings in obj_list and render each .obj then deletes it
scene = bpy.context.scene
scene.frame_set(1)
for item in obj_list:
    path_to_file = os.path.join(path_to_obj_dir, item)
    bpy.ops.import_scene.obj(filepath = path_to_file,axis_forward='Z', axis_up='Y')
    scene.render.image_settings.file_format = 'PNG'
    scene.render.filepath = "D:\\PRIM\\PRIM\\Render\\"  +item[:-4]+ ".png"
    bpy.ops.render.render(write_still = 1)
    name = "frame"
    objs = [obj for obj in bpy.context.visible_objects if name in obj.name]
    bpy.ops.object.select_all(action='DESELECT')
    for o in bpy.context.scene.objects:
        if o.type == 'MESH':
            o.select_set(True)
        else:
            o.select_set(False)
    bpy.ops.object.delete()
    bpy.data.scenes['Scene'].frame_set(bpy.data.scenes['Scene'].frame_current + 1)

