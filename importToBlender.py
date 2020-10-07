import os
import bpy

# put the location to the folder where the objs are located here in this fashion
# this line will only work on windows ie C:\objects
path_to_obj_dir = os.path.join('C:\\Users\\pc\\Desktop\\W\\60', '')

# get list of all files in directory
file_list = sorted(os.listdir(path_to_obj_dir))

# get a list of files ending in 'obj'
obj_list = [item for item in file_list if item.endswith('.obj')]

# loop through the strings in obj_list and add the files to the scene
for item in obj_list:
    path_to_file = os.path.join(path_to_obj_dir, item)
    bpy.ops.import_mesh.obj(filepath = path_to_file,axis_forward='Z', axis_up='Y')
    

def ani_handler(scene):
    name = 'Water'
    objs = [obj for obj in scene.objects.values() if name in obj.name]
    for i, obj in enumerate(objs):
        obj.hide_set(i != ((scene.frame_current-1) % len(objs))) # Blender 2.80
    for i, obj in enumerate(objs):
        obj.hide_render = (i != ((scene.frame_current-1) % len(objs))) # Blender 2.80
        

bpy.app.handlers.frame_change_pre.append(ani_handler)