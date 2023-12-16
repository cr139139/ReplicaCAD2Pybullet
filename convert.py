import os
import sys
import time
import glob
import json
import tqdm
import urdfpy
import trimesh
import argparse
import numpy as np
import pybullet as p
import multiprocessing


def output_path_create(output_path):
    if not os.path.exists(output_path):
        os.makedirs(output_path)


def convert_stages(input_path, output_path):
    stages_path_viz = glob.glob(input_path + '/stages/*.glb')

    pbar = tqdm.tqdm(total=len(stages_path_viz))
    for stage_path_viz in stages_path_viz:
        pbar.set_description('Convert stages')
        pbar.update()
        base_name = os.path.basename(stage_path_viz)[:-4]

        if not os.path.exists(output_path + '/stages/' + base_name):
            os.makedirs(output_path + '/stages/' + base_name)
            os.makedirs(output_path + '/stages/' + base_name + '/visual')

        visual_mesh = trimesh.load(stage_path_viz, force='mesh', skip_texture=False)
        visual_mesh.export(output_path + '/stages/' + base_name + '/visual/' + base_name + '.obj', file_type='glb')
        mesh_viz = urdfpy.Mesh('./visual/' + base_name + '.obj', meshes=[visual_mesh])
        geometry_viz = urdfpy.Geometry(mesh=mesh_viz)
        material = urdfpy.Material(name='white', color=[1, 1, 1, 1])
        visual = urdfpy.Visual(geometry_viz, material=material)
        collision = urdfpy.Collision(name=None, origin=None, geometry=geometry_viz)

        link = urdfpy.Link(base_name, None, [visual], [collision])
        urdf = urdfpy.URDF(base_name, links=[link])
        urdf.save(output_path + '/stages/' + base_name + '/model.urdf')


def convert_dynamic(input_path, output_path):
    dynamics_path_viz = os.listdir(input_path + '/urdf/')

    pbar = tqdm.tqdm(total=len(dynamics_path_viz))
    for dynamic_path_viz in dynamics_path_viz:
        pbar.set_description('Convert dynamic objects')
        pbar.update()
        if not os.path.exists(output_path + '/urdf/' + dynamic_path_viz):
            os.makedirs(output_path + '/urdf/' + dynamic_path_viz)

        for glb in glob.glob(input_path + '/urdf/' + dynamic_path_viz + '/*.glb'):
            base_name = os.path.basename(glb)[:-4]
            if not os.path.exists(output_path + '/urdf/' + dynamic_path_viz + '/' + base_name):
                os.makedirs(output_path + '/urdf/' + dynamic_path_viz + '/' + base_name)
            visual_mesh = trimesh.load(glb, force='mesh', skip_texture=False)
            visual_mesh.export(output_path + '/urdf/' + dynamic_path_viz + '/' + base_name + '/model.obj',
                               file_type='obj')

        for urdf in glob.glob(input_path + '/urdf/' + dynamic_path_viz + '/*.urdf'):
            base_name = os.path.basename(urdf)[:-5]
            object = urdfpy.URDF.load(urdf)
            for link in object.links:
                for visual in link.visuals:
                    visual.geometry.mesh.filename = visual.geometry.mesh.filename[:-4] + '/model.obj'
                    visual.geometry.mesh.meshes = [
                        trimesh.load(output_path + '/urdf/' + dynamic_path_viz + '/' + visual.geometry.mesh.filename,
                                     force='mesh', skip_texture=False)]
                    visual.geometry.mesh.scale = np.ones(3) * np.array(visual.geometry.mesh.scale)
                    visual.material = urdfpy.Material(name='white', color=[1, 1, 1, 1])
            object.save(output_path + '/urdf/' + dynamic_path_viz + '/' + base_name + '.urdf')


def convert_static(input_path, output_path):
    static_objects_path_viz = glob.glob(input_path + '/objects/*.glb')

    pbar = tqdm.tqdm(total=len(static_objects_path_viz))
    for static_object_path_viz in static_objects_path_viz:
        pbar.set_description('Convert static objects')
        pbar.update()
        base_name = os.path.basename(static_object_path_viz)[:-4]
        static_object_path_col = input_path + '/objects/convex/' + base_name + '_cv_decomp.glb'
        static_object_path_cfg = input_path + '/configs/objects/' + base_name + '.object_config.json'

        collision_mesh_exists = True
        if not os.path.exists(static_object_path_col):
            collision_mesh_exists = False

        cfg_json = json.load(open(static_object_path_cfg))
        COM = cfg_json['COM']
        semantic_id = cfg_json['semantic_id']

        mass = 0.
        if 'mass' in cfg_json:
            mass = cfg_json['mass']

        if not os.path.exists(output_path + '/objects/' + base_name):
            os.makedirs(output_path + '/objects/' + base_name)
            os.makedirs(output_path + '/objects/' + base_name + '/visual')
            if collision_mesh_exists:
                os.makedirs(output_path + '/objects/' + base_name + '/collision')

        visual_mesh = trimesh.load(static_object_path_viz, force='mesh', skip_texture=False)
        visual_mesh.export(output_path + '/objects/' + base_name + '/visual/' + base_name + '.obj', file_type='glb')
        mesh_viz = urdfpy.Mesh('./visual/' + base_name + '.obj', meshes=[visual_mesh])
        geometry_viz = urdfpy.Geometry(mesh=mesh_viz)
        material = urdfpy.Material(name=None, color=[1, 1, 1, 1])
        visual = urdfpy.Visual(geometry_viz, material=material)

        if collision_mesh_exists:
            collision_mesh = trimesh.load(static_object_path_col, force='mesh', skip_texture=True)
            collision_mesh.export(output_path + '/objects/' + base_name + '/collision/' + base_name + '.obj',
                                  file_type='glb')
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
        urdf.save(output_path + '/objects/' + base_name + '/model.urdf')


def stage_concave(output_path):
    urdfs_path = glob.glob(output_path + '/stages/*/*.urdf')

    pbar = tqdm.tqdm(total=len(urdfs_path))
    for urdf_path in urdfs_path:
        pbar.set_description('Convert stage urdf to concave')
        pbar.update()
        with open(urdf_path, 'r') as file:
            data = file.read()
            data = data.replace('<collision>', '<collision concave="yes">')
        with open(urdf_path, 'w') as file:
            file.write(data)


def static_convex_decomposition(output_path):
    class HideOutput(object):
        '''
        A context manager that block stdout for its scope, usage:

        with HideOutput():
            os.system('ls -l')
        '''

        def __init__(self, *args, **kw):
            sys.stdout.flush()
            self._origstdout = sys.stdout
            self._oldstdout_fno = os.dup(sys.stdout.fileno())
            self._devnull = os.open(os.devnull, os.O_WRONLY)

        def __enter__(self):
            self._newstdout = os.dup(1)
            os.dup2(self._devnull, 1)
            os.close(self._devnull)
            sys.stdout = os.fdopen(self._newstdout, 'w')

        def __exit__(self, exc_type, exc_val, exc_tb):
            sys.stdout = self._origstdout
            sys.stdout.flush()
            os.dup2(self._oldstdout_fno, 1)

    urdfs_path = glob.glob(output_path + '/objects/*/collision/*.obj')

    pbar = tqdm.tqdm(total=len(urdfs_path))
    for urdf_path in urdfs_path:
        pbar.set_description('Convert object obj to convex')
        pbar.update()
        name_in = urdf_path
        name_out = urdf_path
        name_log = urdf_path[:-4] + ".txt"
        with HideOutput():
            p.vhacd(name_in, name_out, name_log)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=str, required=False, default='ReplicaCAD_dataset')
    parser.add_argument('--output', type=str, required=False, default='ReplicaCAD_urdf')
    parser.add_argument('--viz', type=bool, required=False, default=True)
    parser.add_argument('--viz_path', type=str, required=False,
                        default='./ReplicaCAD_dataset/configs/scenes/apt_0.scene_instance.json')
    args = parser.parse_args()

    input_path = args.input
    output_path = args.output
    start_time = time.time()
    print('__________________________')
    print('Converting process started')
    print('__________________________')

    output_path_create(output_path)

    p1 = multiprocessing.Process(target=convert_stages, args=(input_path, output_path))
    p2 = multiprocessing.Process(target=convert_static, args=(input_path, output_path))
    p3 = multiprocessing.Process(target=convert_dynamic, args=(input_path, output_path))
    p4 = multiprocessing.Process(target=stage_concave, args=(output_path,))
    p5 = multiprocessing.Process(target=static_convex_decomposition, args=(output_path,))

    p1.start()
    p2.start()
    p3.start()

    p1.join()
    p4.start()

    p2.join()
    p5.start()

    p3.join()
    p4.join()
    p5.join()

    print('__________________________')
    print('Converting process ended')
    print("{:4.1f} seconds".format(time.time()-start_time))
    print('__________________________')

    if args.viz:
        from ReplicaCAD2Pybullet.ReplicaCAD_load_pybullet import load_env_from_json
        load_env_from_json(args.viz_path, output_path)
