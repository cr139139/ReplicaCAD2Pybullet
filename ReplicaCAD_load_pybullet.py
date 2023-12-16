import os
import glob
import json
import pybullet as p
import numpy as np
import time


def load_env_from_json(scene_json_path, output_path):
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.setGravity(0, 0, -9.81)
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

    def coordinate_conversion(t, q):
        m = p.getMatrixFromQuaternion(q[1:] + q[0:1])
        m = np.array(m).reshape((3, 3))
        r = np.array([[1, 0, 0],
                      [0, 0, -1],
                      [0, 1, 0]])
        m = r @ m
        w = np.matrix.trace(m)
        q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        w = np.sqrt(w + 1)
        q[3] = 0.5 * w
        w = 0.5 / w
        q[0] = (m[2, 1] - m[1, 2]) * w
        q[1] = (m[0, 2] - m[2, 0]) * w
        q[2] = (m[1, 0] - m[0, 1]) * w
        return (r @ np.array(t)).tolist(), q.tolist()

    translation, rotation = coordinate_conversion(translation, rotation)

    p.loadURDF(output_path + "/stages/" + template_name + "/model.urdf", useFixedBase=True, basePosition=translation,
               baseOrientation=rotation,
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

        translation, rotation = coordinate_conversion(translation, rotation)

        p.loadURDF(output_path + "/objects/" + template_name + "/model.urdf", useFixedBase=useFixedBase,
                   basePosition=translation, baseOrientation=rotation)

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

        translation, rotation = coordinate_conversion(translation, rotation)

        file = glob.glob(output_path + "/urdf/*/" + template_name + ".urdf")[0]
        p.loadURDF(file, useFixedBase=fixed_base,
                   basePosition=translation, baseOrientation=rotation,
                   globalScaling=uniform_scale)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    while True:
        p.stepSimulation()
        time.sleep(1 / 1000.)
    p.disconnect()
