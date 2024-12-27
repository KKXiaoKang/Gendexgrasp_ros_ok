#!bin/bash
# This script is used to run inference on the grasp dataset
## contactdb+cylinder_medium
OBJECT=$1
BATCHSIZE=$2
EXPNAME=$3

## 1. generate contact maps for given objects
python inf_cvae.py --pre_process sharp_lift --s_model PointNetCVAE_Tabletop --num_per_object 32 --comment ${EXPNAME}.cmap

## 2. synthesis grasping poses with generated contact maps
python run_grasp_gen.py --robot_name lejuhand --max_iter 100 --num_particles ${BATCHSIZE} --learning_rate 5e-3 --init_rand_scale 0.5 --object_name ${OBJECT} --cmap_dir logs_inf_cvae/PointNetCVAE_Tabletop/sharp_lift/${EXPNAME}.cmap --comment ${EXPNAME}.poses

## 3. post-process the poses
python preprocess/dict2list.py --original_json logs_gen/leju-${EXPNAME}.poses/align_dist/grasppose@${OBJECT}.json

python refine_grasp.py --basedir logs_gen/leju-${EXPNAME}.poses/align_dist --object ${OBJECT} 

