import glob

urdfs_path = glob.glob('./ReplicaCAD_urdf/objects/*/*.urdf')

for urdf_path in urdfs_path:
    with open(urdf_path, 'r') as file:
        data = file.read()
        data = data.replace('<collision>', '<collision concave="yes">')
    with open(urdf_path[:-5]+'_concave.urdf', 'w') as file:
        file.write(data)