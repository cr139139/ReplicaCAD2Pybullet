import os
import glob
import urdfpy
import trimesh

if not os.path.exists('ReplicaCAD_urdf'):
    os.makedirs('ReplicaCAD_urdf')

stages_path_viz = glob.glob('./ReplicaCAD_dataset/stages/*.glb')

for stage_path_viz in stages_path_viz:
    base_name = os.path.basename(stage_path_viz)[:-4]

    if not os.path.exists('./ReplicaCAD_urdf/stages/' + base_name):
        os.makedirs('./ReplicaCAD_urdf/stages/' + base_name)
        os.makedirs('./ReplicaCAD_urdf/stages/' + base_name + '/visual')

    visual_mesh = trimesh.load(stage_path_viz, force='mesh', skip_texture=False)
    visual_mesh.export('./ReplicaCAD_urdf/stages/' + base_name + '/visual/' + base_name + '.obj', file_type='glb')
    mesh_viz = urdfpy.Mesh('./visual/' + base_name + '.obj', meshes=[visual_mesh])
    geometry_viz = urdfpy.Geometry(mesh=mesh_viz)
    material = urdfpy.Material(name='white', color=[1, 1, 1, 1])
    visual = urdfpy.Visual(geometry_viz, material=material)
    collision = urdfpy.Collision(name=None, origin=None, geometry=geometry_viz)

    link = urdfpy.Link(base_name, None, [visual], [collision])
    urdf = urdfpy.URDF(base_name, links=[link])
    urdf.save('./ReplicaCAD_urdf/stages/' + base_name + '/model.urdf')
