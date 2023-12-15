import os
import glob
import urdfpy
import trimesh
import numpy as np

if not os.path.exists('ReplicaCAD_urdf'):
    os.makedirs('ReplicaCAD_urdf')

dynamics_path_viz = os.listdir('./ReplicaCAD_dataset/urdf/')
print(dynamics_path_viz)

for dynamic_path_viz in dynamics_path_viz:
    if not os.path.exists('./ReplicaCAD_urdf/urdf/' + dynamic_path_viz):
        os.makedirs('./ReplicaCAD_urdf/urdf/' + dynamic_path_viz)

    for glb in glob.glob('./ReplicaCAD_dataset/urdf/' + dynamic_path_viz + '/*.glb'):
        base_name = os.path.basename(glb)[:-4]
        if not os.path.exists('./ReplicaCAD_urdf/urdf/' + dynamic_path_viz + '/' + base_name):
            os.makedirs('./ReplicaCAD_urdf/urdf/' + dynamic_path_viz + '/' + base_name)
        visual_mesh = trimesh.load(glb, force='mesh', skip_texture=False)
        visual_mesh.export('./ReplicaCAD_urdf/urdf/' + dynamic_path_viz + '/' + base_name + '/model.obj', file_type='obj')

    for urdf in glob.glob('./ReplicaCAD_dataset/urdf/' + dynamic_path_viz + '/*.urdf'):
        base_name = os.path.basename(urdf)[:-5]
        object = urdfpy.URDF.load(urdf)
        for link in object.links:
            for visual in link.visuals:
                visual.geometry.mesh.filename = visual.geometry.mesh.filename[:-4] + '/model.obj'
                visual.geometry.mesh.meshes = [trimesh.load('./ReplicaCAD_urdf/urdf/' + dynamic_path_viz + '/' + visual.geometry.mesh.filename, force='mesh', skip_texture=False)]
                visual.geometry.mesh.scale = np.ones(3) * np.array(visual.geometry.mesh.scale)
                visual.material = urdfpy.Material(name='white', color=[1, 1, 1, 1])
        object.save('./ReplicaCAD_urdf/urdf/' + dynamic_path_viz + '/' + base_name + '.urdf')
