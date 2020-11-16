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
l0 = [obj for obj in bpy.context.visible_objects if "001" in obj.name]
n = len(l0)
totalFrame = len(objs)//n

#Voodoo magic starts
l = []
for i in range (totalFrame):
    if i <10:
        frameCount = str(0) + str(0) + str(i)      
    elif 10 <= i <100:
        frameCount = str(0) + str(i)     
    else :
        frameCount = str(i)
    l += [obj for obj in bpy.context.visible_objects if frameCount in obj.name]
    for j,obj in enumerate(l):
        obj.hide_set(i != ((scene.frame_current-1) % totalFrame)) 

def ani_handler(scene):
    for k in range(len(l)):
        l[k].hide_set(k//n != ((scene.frame_current-1) % totalFrame))

        
bpy.app.handlers.frame_change_pre.append(ani_handler)