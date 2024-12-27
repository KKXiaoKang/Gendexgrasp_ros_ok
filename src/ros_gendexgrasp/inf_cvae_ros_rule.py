import argparse
import json
import math
import os.path
import time
import sys
import shutil

import trimesh.sample

import torch
import plotly.graph_objects as go
from utils.visualize_plotly import plot_point_cloud, plot_point_cloud_cmap, plot_mesh_from_name
from utils.set_seed import set_global_seed
from torch.utils.tensorboard import SummaryWriter
import trimesh as tm
import torch.nn as nn


import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import ros_numpy
import trimesh
import numpy as np

object_point_cloud = None

def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pre_process', default='sharp_lift', type=str) # 指定预处理方法

    parser.add_argument('--s_model', default='PointNetCVAE_SqrtFullRobots', type=str) # 模型名称

    parser.add_argument('--num_per_object', default=16, type=int) # 每个对象的样本数

    parser.add_argument('--comment', default='debug', type=str) # 备注信息

    # 解析完后返回参数和当前时间戳
    args_ = parser.parse_args()
    tag = str(time.time())
    return args_, tag


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


def point_cloud_callback(point_msg):
    global object_point_cloud
    # 将ROS的PointCloud2消息转换为点云数组
    point_cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(point_msg)
    
    # 提取XYZ坐标
    points_xyz = np.array([[p[0], p[1], p[2]] for p in point_cloud_array])
    # print("points_xyz XYZ坐标 : ", points_xyz)
    
    # 去除重复点
    points_xyz = np.unique(points_xyz, axis=0)
    print("len(points_xyz) : ", len(points_xyz))

    # 检查点云数据是否有效
    if points_xyz.shape[0] < 3:
        rospy.logwarn("点云数据点太少，无法生成有效的三角网格")
        return

    # 创建Trimesh对象
    mesh = trimesh.Trimesh(vertices=points_xyz, process=True)

    # 检查是否成功生成三角网格
    if len(mesh.faces) == 0:
        rospy.logwarn("Trimesh 对象没有有效的面片，无法进行采样")
        return
    
    # 采样并计算法向量
    sampled_points, face_indices = trimesh.sample.sample_surface(mesh, count=2048)
    normals = mesh.face_normals[face_indices]
    print("normals 法向量 : ", normals)

    # 将点云数据和法向量拼接
    object_point_cloud = torch.tensor(np.hstack((sampled_points, normals)), dtype=torch.float32).to(device)


if __name__ == '__main__':
    # 初始化节点 | 创建订阅者
    rospy.init_node('inf_cvae_ros_rule', anonymous=True)
    rospy.Subscriber('/segmented_object_cloud', PointCloud2, point_cloud_callback)

    # 初始化模型
    set_global_seed(seed=42)
    args, time_tag = get_parser()

    pre_process_map = {'sharp_lift': pre_process_sharp_clamp,
                       'identity': identity_map}
    pre_process_contact_map_goal = pre_process_map[args.pre_process]

    logs_basedir = os.path.join('logs_inf_cvae', f'{args.s_model}', f'{args.pre_process}', f'{args.comment}') # logs基础路径
    vis_cmap_dir = os.path.join(logs_basedir, 'vis_cmap_dir') # 可视化保存路径
    cmap_path = os.path.join(logs_basedir, 'cmap.pt') # 加载模型
    os.makedirs(logs_basedir, exist_ok=False)
    os.makedirs(vis_cmap_dir, exist_ok=False)

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

    # 处理每个对象的点云数据
    cmap = []
    while not rospy.is_shutdown():
        if object_point_cloud is None:
            rospy.loginfo("等待点云数据...")
            rospy.sleep(1.0)  # 暂停1秒，避免一直打印日志
            continue  # 跳过当前循环，等待点云数据

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
        vis_data = [plot_point_cloud_cmap(contact_map_goal[:, :3].cpu().detach().numpy(),
                                          contact_map_goal[:, 6].cpu().detach().numpy())]
        object_name = "111"
        fig = go.Figure(data=vis_data)
        fig.write_html(os.path.join(vis_cmap_dir, f'{object_name}.html'))

    # 保存最终数据
    """
        将所有物体的点云数据和接触图值保存在一个.pt文件中，便于后续加载和使用
    """
    torch.save(cmap, cmap_path)
    rospy.loginfo(f"接触图已保存")


