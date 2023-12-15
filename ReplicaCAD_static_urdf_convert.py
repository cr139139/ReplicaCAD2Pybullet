import json
import os
import glob
import urdfpy
import trimesh
import numpy as np

if not os.path.exists('ReplicaCAD_urdf'):
    os.makedirs('ReplicaCAD_urdf')

static_objects_path_viz = glob.glob('./ReplicaCAD_dataset/objects/*.glb')

for static_object_path_viz in static_objects_path_viz:
    base_name = os.path.basename(static_object_path_viz)[:-4]
    static_object_path_col = './ReplicaCAD_dataset/objects/convex/' + base_name + '_cv_decomp.glb'
    static_object_path_cfg = './ReplicaCAD_dataset/configs/objects/' + base_name + '.object_config.json'

    collision_mesh_exists = True
    if not os.path.exists(static_object_path_col):
        collision_mesh_exists = False

    cfg_json = json.load(open(static_object_path_cfg))
    COM = cfg_json['COM']
    semantic_id = cfg_json['semantic_id']

    mass = 0.
    if 'mass' in cfg_json:
        mass = cfg_json['mass']

    if not os.path.exists('./ReplicaCAD_urdf/objects/' + base_name):
        os.makedirs('./ReplicaCAD_urdf/objects/' + base_name)
        os.makedirs('./ReplicaCAD_urdf/objects/' + base_name + '/visual')
        if collision_mesh_exists:
            os.makedirs('./ReplicaCAD_urdf/objects/' + base_name + '/collision')

    visual_mesh = trimesh.load(static_object_path_viz, force='mesh', skip_texture=False)
    visual_mesh.export('./ReplicaCAD_urdf/objects/' + base_name + '/visual/' + base_name + '.obj', file_type='glb')
    mesh_viz = urdfpy.Mesh('./visual/' + base_name + '.obj', meshes=[visual_mesh])
    geometry_viz = urdfpy.Geometry(mesh=mesh_viz)
    material = urdfpy.Material(name=None, color=[1, 1, 1, 1])
    visual = urdfpy.Visual(geometry_viz, material=material)

    if collision_mesh_exists:
        collision_mesh = trimesh.load(static_object_path_col, force='mesh', skip_texture=True)
        collision_mesh.export('./ReplicaCAD_urdf/objects/' + base_name + '/collision/' + base_name + '.obj', file_type='glb')
        mesh_col = urdfpy.Mesh('./collision/' + base_name + '.obj', meshes=[collision_mesh])
        geometry_col = urdfpy.Geometry(mesh=mesh_col)
        collision = urdfpy.Collision(name=None, origin=None, geometry=geometry_col)
    else:
        collision = urdfpy.Collision(name=None, origin=None, geometry=geometry_viz)

    origin = np.eye(4)
    origin[:3, 3] = COM
    inertia = visual_mesh.moment_inertia * mass
    inertial = urdfpy.Inertial(mass, inertia, origin=origin)

    link = urdfpy.Link(base_name, inertial, [visual], [collision])
    urdf = urdfpy.URDF(base_name, links=[link])
    urdf.save('./ReplicaCAD_urdf/objects/' + base_name + '/model.urdf')
