"""
    工厂模式GenDexGrasp的ROS服务端模块

    contract模块： 根据物体的点云自行选择规则然后生成接触图
    Gengrasp模块： 根据物体的接触图生成最优抓取姿态

    指定种子生成接触图 | 生成抓取姿态

    python gendexgrasp_ros_service.py --seed 42 
"""
import argparse
import json
import math
import os.path
import time

import trimesh.sample
import torch
import plotly.graph_objects as go
from utils.visualize_plotly import plot_point_cloud, plot_point_cloud_cmap, plot_mesh_from_name
from utils.set_seed import set_global_seed
from utils_model.AdamGrasp import AdamGrasp
from utils.get_models import get_handmodel
from torch.utils.tensorboard import SummaryWriter
import trimesh as tm
import torch.nn as nn
import numpy as np

import sys
import shutil
import yaml

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix
from sensor_msgs.msg import JointState
import numpy as np


import glob

def get_parser():
    parser = argparse.ArgumentParser()
    # 备注信息
    parser.add_argument('--comment', default='leju', type=str) # 备注信息

    # Contact 模块
    parser.add_argument('--pre_process', default='sharp_lift', type=str)              # 指定预处理方法
    parser.add_argument('--s_model', default='PointNetCVAE_SqrtFullRobots', type=str) # 模型名称
    parser.add_argument('--seed', default='42', type=int)        # 随机种子
    parser.add_argument('--num_per_object', default=0, type=int) # 每个对象的样本数
    
    # Gengrasp 模块
    parser.add_argument('--robot_name', default='lejuhand', type=str)
    parser.add_argument('--cmap_dir', default='logs_ros_inf_cvae/PointNetCVAE_SqrtFullRobots/sharp_lift/leju/contactdb+water_bottle', type=str)
    parser.add_argument('--max_iter', default=100, type=int)
    parser.add_argument('--steps_per_iter', default=1, type=int)
    parser.add_argument('--num_particles', default=32, type=int)
    parser.add_argument('--learning_rate', default=5e-3, type=float)
    parser.add_argument('--init_rand_scale', default=0.5, type=float)
    parser.add_argument('--object_name', default='contactdb+water_bottle', type=str)
    parser.add_argument('--energy_func', default='align_dist', type=str)
    parser.add_argument('--pose_topic', default='/best_grasp_pose', type=str)
    parser.add_argument('--hand_topic', default='/best_hand_pos', type=str)

    # 解析完后返回参数和当前时间戳
    args_ = parser.parse_args()
    tag = str(time.time())
    return args_, tag

""" -------------------------------- contract 模块 功能函数 ------------------------------------------- """
def pre_process_sharp_clamp(contact_map):
    gap_th = 0.5  # delta_th = (1 - gap_th)
    gap_th = min(contact_map.max().item(), gap_th)
    delta_th = (1 - gap_th)
    contact_map[contact_map > 0.4] += delta_th
    # contact_map += delta_th
    contact_map = torch.clamp_max(contact_map, 1.)
    return contact_map


def identity_map(contact_map):
    return contact_map


def gen_contact_map(args):
    """
        根据选取的点云规则，生成物体接触图（生成单个接触图）
    """
    pre_process_map = {'sharp_lift': pre_process_sharp_clamp,
                       'identity': identity_map}
    pre_process_contact_map_goal = pre_process_map[args.pre_process]

    logs_basedir = os.path.join('logs_ros_inf_cvae', f'{args.s_model}', f'{args.pre_process}', f'{args.comment}') # logs基础路径
    vis_cmap_dir = os.path.join(logs_basedir, 'vis_cmap_dir') # 可视化保存路径
    cmap_path = os.path.join(logs_basedir, 'cmap.pt') # 加载模型

    # exist_ok = True | 如果目录存在可以进行覆盖
    os.makedirs(logs_basedir, exist_ok=True)
    os.makedirs(vis_cmap_dir, exist_ok=True)
                
    device = "cuda"
    if args.s_model == 'PointNetCVAE_SqrtFullRobots':
        model_basedir = 'ckpts/SqrtFullRobots'
        from ckpts.SqrtFullRobots.src.utils_model.PointNetCVAE import PointNetCVAE
        model: nn.Module
        model = PointNetCVAE(latent_size=128,
                             encoder_layers_size=[4, 64, 128, 512],
                             decoder_global_feat_size=512,
                             decoder_pointwise_layers_size=[3, 64, 64],
                             decoder_global_layers_size=[64, 128, 512],
                             decoder_decoder_layers_size=[64 + 512 + 128, 512, 64, 64, 1])
        model.load_state_dict(torch.load(os.path.join(model_basedir, 'weights', 'pointnet_cvae_model.pth'))) # 加载CAVE权重文件
        model = model.to(device) # 将模型加载在GPU上，准备开始推理
        model.eval()             # 将模型设置为评估模式， 禁用掉诸如dropout等在训练时才需要的层。评估模式确保模型在推理时的行为一致
        object_list = json.load(open(os.path.join(model_basedir, "objects.json"), 'rb'))['test'] # 加载要生成接触图的模型file name
    else:
        raise NotImplementedError("Unknown model name")


    for object_name in object_list:
        # 处理每个对象的点云数据(保存整体的数据)
        # cmap = []

        print(f'object name: {object_name}')
        # 加载stl模型（三角面模型）
        object_mesh: tm.Trimesh
        object_mesh = tm.load(os.path.join('data/object', object_name.split('+')[0], object_name.split("+")[1],
                                           f'{object_name.split("+")[1]}.stl'))
        
        # 生成接触图的次数
        count_contact_map = 0
        # 采样点云数据(随机采样样本，根据num_per_object生成不同的点云和法向量)
        while not rospy.is_shutdown():
            # 永远只保存一个contrac_map_model
            cmap = []
            i_sample = count_contact_map

            cmap_ood_sample = {'object_name': object_name,
                              'i_sample': i_sample,
                              'object_point_cloud': None,
                              'contact_map_value': None}
            print(f'[{i_sample}/{args.num_per_object}] | {object_name}')
            # 采样点云和计算法向量
            """
                (1) 使用trimesh的sample_surface方法从物体网格表面采样2048个点，生成点云数据object_point_cloud，并记录每个点对应的面片索引faces_indices
                (2) 从面片索引中提取法向量（face_normals），并将其与点云数据拼接，形成一个包含点的坐标和法向量的点云数据矩阵
            """
            # 随机采样整个网格的面片索引2048个
            object_point_cloud, faces_indices = trimesh.sample.sample_surface(mesh=object_mesh, count=2048)
            # 计算这些采样点对应的法向量
            contact_points_normal = torch.tensor([object_mesh.face_normals[x] for x in faces_indices]).float()
            # 将点云数据转换为 PyTorch Tensor
            object_point_cloud = torch.Tensor(object_point_cloud).float()
            # 筛选特定面片上的点
            # 假设你要筛选 500 到 1001 以及 1500 到 2001 的面片上的点
            # specified_faces_indices = np.concatenate((np.arange(500, 1002), np.arange(1500, 2002)))
            # specified_faces_indices = np.arange(0, 2049)      # 全采样
            specified_faces_indices = np.arange(1500, 2001)
            # 使用布尔掩码筛选 faces_indices 中属于特定面片的点
            mask = np.isin(faces_indices, specified_faces_indices)
            # 重新组成新的局部点云和法向量
            filtered_object_point_cloud = object_point_cloud[mask]
            filtered_contact_points_normal = contact_points_normal[mask]
            # 拼接数据并且传递到设备
            object_point_cloud = torch.cat([filtered_object_point_cloud, filtered_contact_points_normal], dim=1).to(device)

            # 获取整个网格的面片总数
            total_faces_count = len(object_mesh.faces)
            print(f"Total number of faces in the entire mesh: {total_faces_count}")

            # 获取指定面片索引的总数
            specified_faces_count = len(specified_faces_indices)
            print(f"Number of specified faces: {specified_faces_count}")

            # 实际采样的点云数量
            actual_sampled_points_count = object_point_cloud.shape[0]
            print(f"Actual number of sampled points: {actual_sampled_points_count}")

            # 生成潜在代码
            """
                使用标准正态分布随机生成一个潜在代码z_latent_code，它将被输入到解码器中用于生成接触图
            """
            z_latent_code = torch.randn(1, model.latent_size, device=device).float()

            # 通过模型推理接触图
            """
                调用PointNetCVAE模型的inference方法，将采样的点云坐标（去掉法向量部分）和潜在代码输入模型，输出接触图值contact_map_value
            """
            contact_map_value = model.inference(object_point_cloud[:, :3].unsqueeze(0), z_latent_code).squeeze(0)

            # 对接触图值进行后处理
            # process the contact map value
            """
                将接触图值转移到CPU上，并进行必要的后处理（如clamp操作），以确保数值在合适的范围内。
            """
            contact_map_value = contact_map_value.detach().cpu().unsqueeze(1)
            contact_map_value = pre_process_contact_map_goal(contact_map_value).to(device)

            # 拼接接触图与点云数据
            """
                将经过后处理的接触图值与原始点云数据再次拼接，以形成一个完整的数据结构，包含点的坐标、法向量和接触图值
            """
            contact_map_goal = torch.cat([object_point_cloud, contact_map_value], dim=1)

            # 保存数据与可视化
            """
                (1) 将生成的点云数据和接触图值存储在cmap_ood_sample字典中，并将其追加到cmap列表中
                (2) 使用plot_point_cloud_cmap函数可视化点云数据并使用颜色映射展示接触图值，生成一个3D可视化图像
                (3) 最后将可视化图像保存为HTML文件，便于浏览和分析
            """
            cmap_ood_sample['object_point_cloud'] = object_point_cloud
            cmap_ood_sample['contact_map_value'] = contact_map_value
            cmap.append(cmap_ood_sample)
            print(" len(cmap) 长度为：" , len(cmap))

            vis_data = []
            vis_data += [plot_point_cloud_cmap(contact_map_goal[:, :3].cpu().detach().numpy(),
                                               contact_map_goal[:, 6].cpu().detach().numpy())]
            vis_data += [plot_mesh_from_name(f'{object_name}')]
            fig = go.Figure(data=vis_data)
            fig.write_html(os.path.join(vis_cmap_dir, f'{object_name}-{i_sample}.html'))
            
            # 生成该物体的专属 cmap_path
            cmap_path = os.path.join(logs_basedir, object_name, f'{object_name}_sample_{i_sample}.pt')
            
            # 创建目录以确保路径存在
            os.makedirs(os.path.dirname(cmap_path), exist_ok=True)

            # 保存该样本的 cmap_ood_sample 数据到独立的 .pt 文件
            torch.save(cmap_ood_sample, cmap_path)
            print(f'Saved cmap for {object_name}, sample {i_sample} to {cmap_path}')

            # ++
            count_contact_map +=1

""" -------------------------------- contract 模块 功能函数 ------------------------------------------- """

## ---------------------------------------------------------------------------------------------------------- ##

""" -------------------------------- Gengrasp 模块 功能函数 ------------------------------------------- """
# 将 best_q 保存到 config.yaml 文件中
def save_best_q_to_yaml(best_q, object_name, i_sample, yaml_path):
    # 将 best_q 转换为 Python 列表，并创建数据结构
    best_q_list = best_q.cpu().numpy().tolist()
    config_data = {
        f'{object_name}_{i_sample}_best_q': best_q_list
    }
    
    # 检查是否已存在 YAML 文件，如果存在，加载现有数据
    if os.path.exists(yaml_path):
        with open(yaml_path, 'r') as file:
            existing_data = yaml.safe_load(file)
            if existing_data is None:
                existing_data = {}
    else:
        existing_data = {}
    
    # 更新 YAML 数据并保存回文件
    existing_data.update(config_data)
    with open(yaml_path, 'w') as file:
        yaml.safe_dump(existing_data, file)

def best_q_to_posestamped(best_q, frame_id="torso"):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = frame_id

    # 提取平移向量 (best_q的前3个元素)
    translation = best_q[0, :3].cpu().numpy()

    # 提取旋转矩阵的前6个分量
    rotation_matrix_components = best_q[0, 3:9].cpu().numpy()

    # 初始化旋转矩阵
    rotation_matrix = np.zeros((3, 3))

    # 将前两列分量赋值给旋转矩阵的前两列
    rotation_matrix[:, 0] = rotation_matrix_components[:3]
    rotation_matrix[:, 1] = rotation_matrix_components[3:6]

    # 使用Gram-Schmidt过程正交化第三列
    rotation_matrix[:, 1] -= np.dot(rotation_matrix[:, 0], rotation_matrix[:, 1]) * rotation_matrix[:, 0]
    rotation_matrix[:, 1] /= np.linalg.norm(rotation_matrix[:, 1])
    rotation_matrix[:, 2] = np.cross(rotation_matrix[:, 0], rotation_matrix[:, 1])

    # 将3x3旋转矩阵转换为4x4齐次变换矩阵
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix

    # 将旋转矩阵转换为四元数
    quaternion = quaternion_from_matrix(homogeneous_matrix)

    # 填充PoseStamped消息
    pose_msg.pose.position.x = translation[0]
    pose_msg.pose.position.y = translation[1]
    pose_msg.pose.position.z = translation[2]
    pose_msg.pose.orientation.x = quaternion[0]
    pose_msg.pose.orientation.y = quaternion[1]
    pose_msg.pose.orientation.z = quaternion[2]
    pose_msg.pose.orientation.w = quaternion[3]

    return pose_msg

def best_q_to_posehand(best_q, frame_id="torso"):
    # 定义手指关节名称
    joint_names = [
        "base_link", "thumb1", "thumb2", "index1", "index2",
        "middle1", "middle2", "ring1", "ring2", "little1", "little2"
    ]

    # 提取关节位置
    joint_positions = best_q[0, 9:19].cpu().numpy()

    # 创建 JointState 消息
    robot_hand_joint_msg = JointState()
    robot_hand_joint_msg.header.stamp = rospy.Time.now()
    robot_hand_joint_msg.header.frame_id = frame_id
    robot_hand_joint_msg.name = joint_names
    robot_hand_joint_msg.position = joint_positions

    return robot_hand_joint_msg


def gen_grasp_map(args):
    # 设置torch
    torch.set_printoptions(precision=4, sci_mode=False, edgeitems=8)

    # 检查参数
    print(args)
    print(f'double check....')

    # ros工程化
    pose_pub = rospy.Publisher(args.pose_topic, PoseStamped, queue_size=10)     # 发布/best_grasp_pose 用于发布每个接触图下的最优抓取姿态PoseStamped
    hand_pos_pub = rospy.Publisher(args.hand_topic, JointState, queue_size=10) # 发布/best_hand_pos   用于发布接触图下当前手腕位置PoseStamped

    # 设置路径
    logs_basedir = os.path.join('logs_ros_gen', f'leju-{args.comment}', f'{args.energy_func}')
    tb_dir = os.path.join(logs_basedir, 'tb_dir')
    tra_dir = os.path.join(logs_basedir, 'tra_dir')
    vis_dir = os.path.join(logs_basedir, 'vis_dir')
    os.makedirs(logs_basedir, exist_ok=True)
    os.makedirs(tra_dir, exist_ok=True)
    os.makedirs(tb_dir, exist_ok=True)
    os.makedirs(vis_dir, exist_ok=True)
    writer = SummaryWriter(tb_dir)

    f = open(os.path.join(logs_basedir, 'command.txt'), 'w')
    f.write(' '.join(sys.argv))
    f.close()

    src_dir_list = ['utils', 'utils_model', 'utils_data']
    os.makedirs(os.path.join(logs_basedir, 'src'), exist_ok=True)
    for fn in os.listdir('.'):
        if fn[-3:] == '.py':
            shutil.copy(fn, os.path.join(logs_basedir, 'src', fn))
    for src_dir in src_dir_list:
        for fn in os.listdir(f'{src_dir}'):
            os.makedirs(os.path.join(logs_basedir, 'src', f'{src_dir}'), exist_ok=True)
            if fn[-3:] == '.py' or fn[-5:] == '.yaml':
                shutil.copy(os.path.join(f'{src_dir}', fn), os.path.join(logs_basedir, 'src', f'{src_dir}', fn))

    robot_name = args.robot_name
    device = "cuda" if torch.cuda.is_available() else "cpu"
    yaml_path = os.path.join(logs_basedir, 'config.yaml') 

    # 初始化模型
    # init model
    """
        初始化 AdamGrasp 模型，这个模型用于执行基于粒子群优化的抓取生成任务。初始化时需要提供机器人名称、粒子数量、学习率、最大迭代次数等参数
    """
    model = AdamGrasp(robot_name=robot_name, writer=writer, contact_map_goal=None,
                      num_particles=args.num_particles, init_rand_scale=args.init_rand_scale, max_iter=args.max_iter,
                      steps_per_iter=args.steps_per_iter, learning_rate=args.learning_rate, device=device,
                      energy_func_name=args.energy_func)
    
    # 可视化准备
    """
        (1) 初始化手部模型 (handmodel)，用于后续的抓取姿势可视化
        (2) 加载物体的3D网格模型，便于可视化展示抓取效果
    """
    # hand model for visualization
    handmodel = get_handmodel(robot=robot_name, batch_size=1, device=device)
    # object mesh for visualization
    object_vis_data = plot_mesh_from_name(args.object_name)

    # 加载接触图数据集(原本是加载一个完整的pt图进行Adam优化)
    cmap_files = sorted(glob.glob(os.path.join(args.cmap_dir, 'contactdb+water_bottle_sample_*.pt')))
 
    for cmap_file in cmap_files:
        print(f"Processing {cmap_file}...")
        # 加载接触图数据集
        try:
            cmap_dataset = torch.load(cmap_file)
        except FileNotFoundError:
            raise NotImplementedError(f'Could not load {cmap_file}')

        # 处理数据
        """
            (1) 遍历加载的接触图数据集，为每个符合条件的物体和样本生成抓取
            (2) 使用 AdamGrasp 模型的 run_adam 方法执行优化，生成最佳的抓取姿势
        """
        adam_object_name = args.object_name
        object_name = adam_object_name

        object_point_cloud = cmap_dataset['object_point_cloud']
        i_sample = cmap_dataset['i_sample']
        contact_map_value = cmap_dataset['contact_map_value']
        running_name = f'{object_name}+{i_sample}'
        contact_map_goal = torch.cat([object_point_cloud, contact_map_value], dim=1).to(device)
        record = model.run_adam(object_name=object_name, contact_map_goal=contact_map_goal, running_name=running_name)

        with torch.no_grad():
            q_tra, energy, steps_per_iter = record
            i_record = {'q_tra': q_tra[:, -1:, :].detach(),
                        'energy': energy,
                        'steps_per_iter': steps_per_iter,
                        'comment': args.comment,
                        'object_name': object_name,
                        'i_sample': i_sample}
            torch.save(i_record, os.path.join(tra_dir, f'tra-{object_name}-{i_sample}.pt'))

            best_q = q_tra[energy.argmin(), -1, :]
            best_q = best_q.unsqueeze(0)
            save_best_q_to_yaml(best_q, object_name, i_sample, yaml_path)
            handmodel.update_kinematics(q=best_q)
            vis_data = handmodel.get_plotly_data(color='pink', opacity=1.0)
            vis_data += [object_vis_data]
            fig = go.Figure(data=vis_data)
            fig.write_html(os.path.join(vis_dir, f'grasppose-{object_name}-{i_sample}.html'))

            # 将best_q发布为PoseStamped消息
            pose_msg = best_q_to_posestamped(best_q)
            pose_pub.publish(pose_msg)
            rospy.loginfo(f"Published best grasp pose for {object_name}-{i_sample}")

            # 将best_q发布为JointState消息
            hand_msg = best_q_to_posehand(best_q)
            hand_pos_pub.publish(hand_msg)
            rospy.loginfo(f"Published best grasp hand_msg 灵巧手 for {object_name}-{i_sample}")
    
""" -------------------------------- Gengrasp 模块 功能函数 ------------------------------------------- """


""" -------------------------------- 主函数 ------------------------------------------- """
if __name__ == "__main__":  
    
    rospy.init_node('gen_contact_map_node', anonymous=True)

    args, time_tag = get_parser()

    set_global_seed(args.seed)
    
    # gen_contact_map(args)

    gen_grasp_map(args)