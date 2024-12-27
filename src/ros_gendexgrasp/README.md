# GenDexGrasp for LejuHand Grasp Pose Generation

This is a code repo for lejuhand grasping pose generation given an object mesh, which is built on [GenDexGrasp](https://github.com/tengyu-liu/GenDexGrasp).

## Conda Env Preparation

- Create a conda environment with python 3.8
  
  ```bash
  conda create -n gendexgrasp python==3.8
  conda activate gendexgrasp
  ```

- Install pytorch and cudatoolkit with a specific version
  
    ```bash
    conda install pytorch==1.12.0 torchvision==0.13.0 torchaudio==0.12.0 cudatoolkit=11.3 -c pytorch
    ```

- Install other dependencies
      pip install -r requirements.txt
  
- Install modified thirdparty dependencies

    ```bash
    cd thirdparty/pytorch_kinematics
    pip install -e .
    pip install urdf-parser-py
    ```



## Example for Usage

### 0. Prepare the given object meshes

For example, we prepare the object meshes in `./data/objects/` for the following steps. Please oganize the object meshes in the following format:

  ```
  ./data/object/
  ├── contactdb
  │   ├── apple
  │   │   ├── apple.stl
  │   ├── camera
  │   │   ├── camera.stl
  │   ├── contactdb.json
  ├── ${DATASET-NAME}
  │   ├── ${OBJECT-NAME}
  │   │   ├── ${OBJECT-NAME}.stl
  │   ├── ${DATASET-NAME}.json
  ```

And we refer to the object mesh with the name `${DATASET-NAME}+${OBJECT-NAME}` (e.g., `contactdb+apple`) for the following steps.

### 1. Generate contact maps for given objects

We prepare a pre-trained weight [here](./ckpts/SqrtFullRobots/weights) for contact map generation model.

Rewrite the `objects.json` file in pre-trained model base folder (e.g., `./ckpts/SqrtFullRobots/`) with the given object names (e.g., `contactdb+apple`), then run

```bash
python inf_cvae.py --pre_process sharp_lift --s_model PointNetCVAE_SqrtFullRobots --num_per_object 16 --comment leju
```

### 2. Synthesis grasping poses with generated contact maps

Assume the generated contact maps (named with `cmap.pt` by default) are saved in `logs_inf_cvae/PointNetCVAE_SqrtFullRobots/sharp_lift/leju/`, run

```bash
python run_grasp_gen.py --robot_name lejuhand --max_iter 100 --num_particles 32 --learning_rate 5e-3 --init_rand_scale 0.5 --object_name contactdb+apple --cmap_dir logs_inf_cvae/PointNetCVAE_SqrtFullRobots/sharp_lift/leju
```

### 3. Visualize the generated grasping poses

Directing to the log folder made by the previous step (e.g., `logs_gen/leju-default/align_dist`), visualization results are saved in `./vis_dir`.



## Citation

If you find this work is helpful, please consider citing us as

```tex
@inproceedings{li2023gendexgrasp,
  title={Gendexgrasp: Generalizable dexterous grasping},
  author={Li, Puhao and Liu, Tengyu and Li, Yuyang and Geng, Yiran and Zhu, Yixin and Yang, Yaodong and Huang, Siyuan},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8068--8074},
  year={2023},
  organization={IEEE}
}
```
```bash
Mesh resource marker [/0] could not load [file:///home/lab/GenDexGrasp/Gendexgrasp_ros/src/ros_robot_model/biped_s4/models/biped_s4/meshes/r_hand_roll.obj

Mesh resource marker [/0] could not load [file:///home/lab/GenDexGrasp/Gendexgrasp_ros/src/ros_robot_model/biped_s4/meshes/r_hand_roll.obj]
```

## for water_bottle_grasp
```bash
# 生成抓取图（根据指定的面片索引进行接触图的生成）
python inf_cvae.py --pre_process sharp_lift --s_model PointNetCVAE_SqrtFullRobots --num_per_object 2 --comment leju

# 根据抓取图不断生成抓取姿态ros信息
python run_grasp_gen_ros.py --robot_name lejuhand --max_iter 100 --num_particles 32 --learning_rate 5e-3 --init_rand_scale 0.5 --object_name contactdb+water_bottle --cmap_dir logs_inf_cvae/PointNetCVAE_SqrtFullRobots/sharp_lift/leju
```