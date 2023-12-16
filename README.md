# ReplicaCAD2Pybullet

This is a Python library for converting RelicaCAD datasets to be able to be imported into the Pybullet environment.

## Usage

1. Install [ReplicaCAD datasets](https://huggingface.co/datasets/ai-habitat/ReplicaCAD_dataset)
```terminal
git lfs install
git clone https://huggingface.co/datasets/ai-habitat/ReplicaCAD_dataset
```
   
2. Clone this repo and run the following code
```terminal
python convert.py --input {ReplicaCAD path} --output {desired export path} --viz True --viz_path {ReplicaCAD config scene json path}
```

Example
```terminal
python convert.py --input ../ReplicaCAD_dataset --output ../ReplicaCAD_new --viz True --viz_path ../ReplicaCAD_dataset/configs/scenes/apt_0.scene_instance.json
```