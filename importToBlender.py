import os
import bpy

# put the location to the folder where the objs are located here in this fashion
# this line will only work on windows ie C:\objects
path_to_obj_dir = os.path.join('D:\\PRIM\\PRIM\\Output', '')

# get list of all files in directory
file_list = sorted(os.listdir(path_to_obj_dir))

# get a list of files ending in 'obj'
obj_list = [item for item in file_list if item.endswith('.obj')]

# loop through the strings in obj_list and add the files to the scene
for item in obj_list:
    path_to_file = os.path.join(path_to_obj_dir, item)
    bpy.ops.import_scene.obj(filepath = path_to_file,axis_forward='Z', axis_up='Y')

#set a frame for each .obj file in both set and render modes    
scene = bpy.context.scene
name = "frame"
objs = [obj for obj in bpy.context.visible_objects if name in obj.name]
for i, obj in enumerate(objs):
    obj.hide_set(i != ((scene.frame_current-1) % len(objs))) 
    obj.hide_render = (i != ((scene.frame_current-1) % len(objs)))
def ani_handler(scene):
        for i, obj in enumerate(objs):
            obj.hide_set(i != ((scene.frame_current-1) % len(objs))) 
            obj.hide_render = (i != ((scene.frame_current-1) % len(objs))) 

bpy.app.handlers.frame_change_pre.append(ani_handler)