import os
import glob
import json
import pybullet as p
import numpy as np
import time

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
scene_json_path = './ReplicaCAD_dataset/configs/scenes/apt_0.scene_instance.json'
scene_json = json.load(open(scene_json_path))

stage_json = scene_json['stage_instance']
template_name = stage_json['template_name']
template_name = os.path.basename(template_name)
translation = [0, 0, 0]
if 'traslation' in stage_json:
    translation = stage_json['translation']
rotation = [1, 0, 0, 0]
if 'rotation' in stage_json:
    rotation = stage_json['rotation']
translation_origin = [0, 0, 0]
if 'translation_origin' in stage_json:
    translation_origin = stage_json['translation_origin']


def rotationMatrixToQuaternion1(m):
    # q0 = qw
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    t = np.sqrt(t + 1)
    q[0] = 0.5 * t
    t = 0.5 / t
    q[1] = (m[2, 1] - m[1, 2]) * t
    q[2] = (m[0, 2] - m[2, 0]) * t
    q[3] = (m[1, 0] - m[0, 1]) * t
    return q


m = p.getMatrixFromQuaternion(rotation[1:] + rotation[0:1])
m = np.array(m).reshape((3, 3))
r = np.array([[1, 0, 0],
              [0, 0, -1],
              [0, 1, 0]])
rotation = rotationMatrixToQuaternion1(r @ m).tolist()
translation = r @ np.array(translation)

p.loadURDF("./ReplicaCAD_urdf/stages/" + template_name + "/model.urdf", useFixedBase=True, basePosition=translation,
           baseOrientation=rotation[1:] + rotation[0:1],
           flags=p.GEOM_FORCE_CONCAVE_TRIMESH)

for object in scene_json['object_instances']:
    template_name = object['template_name']
    template_name = os.path.basename(template_name)
    translation = object['translation']
    rotation = object['rotation']
    motion_type = object['motion_type']
    useFixedBase = True
    if motion_type == 'DYNAMIC':
        useFixedBase = False
    translation_origin = [0, 0, 0]
    if 'translation_origin' in object:
        translation_origin = object['translation_origin']

    m = p.getMatrixFromQuaternion(rotation[1:] + rotation[0:1])
    m = np.array(m).reshape((3, 3))
    r = np.array([[1, 0, 0],
                  [0, 0, -1],
                  [0, 1, 0]])
    rotation = rotationMatrixToQuaternion1(r @ m).tolist()
    translation = r @ np.array(translation)

    if useFixedBase:
        p.loadURDF("./ReplicaCAD_urdf/objects/" + template_name + "/model_concave.urdf", useFixedBase=useFixedBase,
                   basePosition=translation,
                   baseOrientation=rotation[1:] + rotation[0:1])
    else:
        p.loadURDF("./ReplicaCAD_urdf/objects/" + template_name + "/model.urdf", useFixedBase=useFixedBase,
                   basePosition=translation,
                   baseOrientation=rotation[1:] + rotation[0:1])

for object in scene_json['articulated_object_instances']:
    template_name = object['template_name']
    template_name = os.path.basename(template_name)
    translation = object['translation']
    rotation = object['rotation']
    motion_type = object['motion_type']
    translation_origin = [0, 0, 0]
    if 'translation_origin' in object:
        translation_origin = object['translation_origin']
    fixed_base = object['fixed_base']
    auto_clamp_joint_limits = False
    if 'auto_clamp_joint_limits' in object:
        auto_clamp_joint_limits = object['auto_clamp_joint_limits']
    uniform_scale = 1.0
    if 'uniform_scale' in object:
        uniform_scale = object['uniform_scale']

    m = p.getMatrixFromQuaternion(rotation[1:] + rotation[0:1])
    m = np.array(m).reshape((3, 3))
    r = np.array([[1, 0, 0],
                  [0, 0, -1],
                  [0, 1, 0]])
    rotation = rotationMatrixToQuaternion1(r @ m).tolist()
    translation = r @ np.array(translation)

    file = glob.glob("./ReplicaCAD_urdf/urdf/*/" + template_name + ".urdf")[0]
    p.loadURDF(file, useFixedBase=fixed_base,
               basePosition=translation,
               baseOrientation=rotation[1:] + rotation[0:1],
               globalScaling=uniform_scale)

while True:
    p.stepSimulation()
    time.sleep(1 / 1000.)
p.disconnect()
