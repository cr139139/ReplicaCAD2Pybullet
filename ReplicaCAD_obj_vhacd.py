import glob
import tqdm
import pybullet as p
urdfs_path = glob.glob('./ReplicaCAD_urdf/objects/*/collision/*.obj')

for urdf_path in tqdm.tqdm(urdfs_path):
    name_in = urdf_path
    name_out = urdf_path
    name_log = urdf_path[:-4] + ".txt"
    p.vhacd(name_in, name_out, name_log)