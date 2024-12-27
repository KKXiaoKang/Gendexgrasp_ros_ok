# import argparse
# import json
# import math
# import os.path
# import time
# import sys
# import shutil

# import trimesh.sample

# import torch
# import plotly.graph_objects as go
# from utils.visualize_plotly import plot_point_cloud, plot_point_cloud_cmap, plot_mesh_from_name
# from utils.set_seed import set_global_seed
# from torch.utils.tensorboard import SummaryWriter
# import trimesh as tm
# import torch.nn as nn
# import numpy as np

# def get_parser():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--pre_process', default='sharp_lift', type=str) # 指定预处理方法

#     parser.add_argument('--s_model', default='PointNetCVAE_SqrtFullRobots', type=str) # 模型名称

#     parser.add_argument('--num_per_object', default=16, type=int) # 每个对象的样本数

#     parser.add_argument('--comment', default='debug', type=str) # 备注信息

#     # 解析完后返回参数和当前时间戳
#     args_ = parser.parse_args()
#     tag = str(time.time())
#     return args_, tag


# def pre_process_sharp_clamp(contact_map):
#     gap_th = 0.5  # delta_th = (1 - gap_th)
#     gap_th = min(contact_map.max().item(), gap_th)
#     delta_th = (1 - gap_th)
#     contact_map[contact_map > 0.4] += delta_th
#     # contact_map += delta_th
#     contact_map = torch.clamp_max(contact_map, 1.)
#     return contact_map


# def identity_map(contact_map):
#     return contact_map


# if __name__ == '__main__':
#     set_global_seed(seed=42)
#     args, time_tag = get_parser()

#     pre_process_map = {'sharp_lift': pre_process_sharp_clamp,
#                        'identity': identity_map}
#     pre_process_contact_map_goal = pre_process_map[args.pre_process]

#     logs_basedir = os.path.join('logs_inf_cvae', f'{args.s_model}', f'{args.pre_process}', f'{args.comment}') # logs基础路径
#     vis_cmap_dir = os.path.join(logs_basedir, 'vis_cmap_dir') # 可视化保存路径
#     cmap_path = os.path.join(logs_basedir, 'cmap.pt') # 加载模型
#     os.makedirs(logs_basedir, exist_ok=False)
#     os.makedirs(vis_cmap_dir, exist_ok=False)

#     device = "cuda"
#     if args.s_model == 'PointNetCVAE_SqrtFullRobots':
#         model_basedir = 'ckpts/SqrtFullRobots'
#         from ckpts.SqrtFullRobots.src.utils_model.PointNetCVAE import PointNetCVAE
#         model: nn.Module
#         model = PointNetCVAE(latent_size=128,
#                              encoder_layers_size=[4, 64, 128, 512],
#                              decoder_global_feat_size=512,
#                              decoder_pointwise_layers_size=[3, 64, 64],
#                              decoder_global_layers_size=[64, 128, 512],
#                              decoder_decoder_layers_size=[64 + 512 + 128, 512, 64, 64, 1])
#         model.load_state_dict(torch.load(os.path.join(model_basedir, 'weights', 'pointnet_cvae_model.pth'))) # 加载CAVE权重文件
#         model = model.to(device) # 将模型加载在GPU上，准备开始推理
#         model.eval()             # 将模型设置为评估模式， 禁用掉诸如dropout等在训练时才需要的层。评估模式确保模型在推理时的行为一致
#         object_list = json.load(open(os.path.join(model_basedir, "objects.json"), 'rb'))['test'] # 加载要生成接触图的模型file name
#     else:
#         raise NotImplementedError("Unknown model name")

#     # 处理每个对象的点云数据
#     cmap = []
#     for object_name in object_list:
#         print(f'object name: {object_name}')
#         # 加载stl模型（三角面模型）
#         object_mesh: tm.Trimesh
#         object_mesh = tm.load(os.path.join('data/object', object_name.split('+')[0], object_name.split("+")[1],
#                                            f'{object_name.split("+")[1]}.stl'))
#         # 采样点云数据(随机采样样本，根据num_per_object生成不同的点云和法向量)
#         for i_sample in range(args.num_per_object):
#             cmap_ood_sample = {'object_name': object_name,
#                               'i_sample': i_sample,
#                               'object_point_cloud': None,
#                               'contact_map_value': None}
#             print(f'[{i_sample}/{args.num_per_object}] | {object_name}')
#             # 采样点云和计算法向量
#             """
#                 (1) 使用trimesh的sample_surface方法从物体网格表面采样2048个点，生成点云数据object_point_cloud，并记录每个点对应的面片索引faces_indices
#                 (2) 从面片索引中提取法向量（face_normals），并将其与点云数据拼接，形成一个包含点的坐标和法向量的点云数据矩阵
#             """
#             # 随机采样整个网格的面片索引2048个
#             object_point_cloud, faces_indices = trimesh.sample.sample_surface(mesh=object_mesh, count=2048)
#             # 计算这些采样点对应的法向量
#             contact_points_normal = torch.tensor([object_mesh.face_normals[x] for x in faces_indices]).float()
#             # 将点云数据转换为 PyTorch Tensor
#             object_point_cloud = torch.Tensor(object_point_cloud).float()
#             # 筛选特定面片上的点
#             # 假设你要筛选 500 到 1001 以及 1500 到 2001 的面片上的点
#             # specified_faces_indices = np.concatenate((np.arange(500, 1002), np.arange(1500, 2002)))
#             # specified_faces_indices = np.arange(0, 2049)      # 全采样
#             specified_faces_indices = np.arange(1500, 2001)
#             # 使用布尔掩码筛选 faces_indices 中属于特定面片的点
#             mask = np.isin(faces_indices, specified_faces_indices)
#             # 重新组成新的局部点云和法向量
#             filtered_object_point_cloud = object_point_cloud[mask]
#             filtered_contact_points_normal = contact_points_normal[mask]
#             # 拼接数据并且传递到设备
#             object_point_cloud = torch.cat([filtered_object_point_cloud, filtered_contact_points_normal], dim=1).to(device)

#             # # 下面的算法因为是一开始就对面片索引进行提取，然后再通过sample_surface进行点云生成，这样子会导致有些时候有些物体的三角面点云本身瓶子中间缺失信息，但是头部和尾部集中了信息，不能均匀的取到点云
#             # # 1. 确定特定的面片索引
#             # #specified_faces_indices = np.arange(500, 1001)  # 500到1000的面片索引，包含1000
#             # #specified_faces_indices = np.arange(1000, 2001)
#             # range_1 = np.arange(500, 1002)  # 包括 1001
#             # range_2 = np.arange(1500, 2002)  # 包括 2001
#             # specified_faces_indices = np.concatenate((range_1, range_2))
#             # # 2. 提取这些面片的顶点
#             # specified_faces = object_mesh.faces[specified_faces_indices]
#             # # 3. 创建局部子网格
#             # specified_mesh_list = object_mesh.submesh([specified_faces_indices], only_watertight=False)
#             # # 4. 提取子网格（submesh返回的是列表）
#             # specified_mesh = specified_mesh_list[0]
#             # # 5. 在局部子网格上采样点云
#             # object_point_cloud, faces_indices = trimesh.sample.sample_surface(mesh=specified_mesh, count=2048)
#             # # 6. 计算法向量并拼接
#             # contact_points_normal = torch.tensor([specified_mesh.face_normals[x] for x in faces_indices]).float()
#             # object_point_cloud = torch.Tensor(object_point_cloud).float()
#             # object_point_cloud = torch.cat([object_point_cloud, contact_points_normal], dim=1).to(device)

#             # 获取整个网格的面片总数
#             total_faces_count = len(object_mesh.faces)
#             print(f"Total number of faces in the entire mesh: {total_faces_count}")

#             # 获取指定面片索引的总数
#             specified_faces_count = len(specified_faces_indices)
#             print(f"Number of specified faces: {specified_faces_count}")

#             # 实际采样的点云数量
#             actual_sampled_points_count = object_point_cloud.shape[0]
#             print(f"Actual number of sampled points: {actual_sampled_points_count}")

#             # 生成潜在代码
#             """
#                 使用标准正态分布随机生成一个潜在代码z_latent_code，它将被输入到解码器中用于生成接触图
#             """
#             z_latent_code = torch.randn(1, model.latent_size, device=device).float()

#             # 通过模型推理接触图
#             """
#                 调用PointNetCVAE模型的inference方法，将采样的点云坐标（去掉法向量部分）和潜在代码输入模型，输出接触图值contact_map_value
#             """
#             contact_map_value = model.inference(object_point_cloud[:, :3].unsqueeze(0), z_latent_code).squeeze(0)

#             # 对接触图值进行后处理
#             # process the contact map value
#             """
#                 将接触图值转移到CPU上，并进行必要的后处理（如clamp操作），以确保数值在合适的范围内。
#             """
#             contact_map_value = contact_map_value.detach().cpu().unsqueeze(1)
#             contact_map_value = pre_process_contact_map_goal(contact_map_value).to(device)

#             # 拼接接触图与点云数据
#             """
#                 将经过后处理的接触图值与原始点云数据再次拼接，以形成一个完整的数据结构，包含点的坐标、法向量和接触图值
#             """
#             contact_map_goal = torch.cat([object_point_cloud, contact_map_value], dim=1)

#             # 保存数据与可视化
#             """
#                 (1) 将生成的点云数据和接触图值存储在cmap_ood_sample字典中，并将其追加到cmap列表中
#                 (2) 使用plot_point_cloud_cmap函数可视化点云数据并使用颜色映射展示接触图值，生成一个3D可视化图像
#                 (3) 最后将可视化图像保存为HTML文件，便于浏览和分析
#             """
#             cmap_ood_sample['object_point_cloud'] = object_point_cloud
#             cmap_ood_sample['contact_map_value'] = contact_map_value
#             cmap.append(cmap_ood_sample)
#             vis_data = []
#             vis_data += [plot_point_cloud_cmap(contact_map_goal[:, :3].cpu().detach().numpy(),
#                                                contact_map_goal[:, 6].cpu().detach().numpy())]
#             vis_data += [plot_mesh_from_name(f'{object_name}')]
#             fig = go.Figure(data=vis_data)
#             fig.write_html(os.path.join(vis_cmap_dir, f'{object_name}-{i_sample}.html'))
#     # 保存最终数据
#     """
#         将所有物体的点云数据和接触图值保存在一个.pt文件中，便于后续加载和使用
#     """
#     torch.save(cmap, cmap_path)
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


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pre_process', default='sharp_lift', type=str)

    parser.add_argument('--s_model', default='PointNetCVAE_SqrtFullRobots', type=str)

    parser.add_argument('--num_per_object', default=16, type=int)

    parser.add_argument('--comment', default='debug', type=str)
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


if __name__ == '__main__':
    set_global_seed(seed=42)
    args, time_tag = get_parser()

    pre_process_map = {'sharp_lift': pre_process_sharp_clamp,
                       'identity': identity_map}
    pre_process_contact_map_goal = pre_process_map[args.pre_process]

    logs_basedir = os.path.join('logs_inf_cvae', f'{args.s_model}', f'{args.pre_process}', f'{args.comment}')
    vis_cmap_dir = os.path.join(logs_basedir, 'vis_cmap_dir')
    cmap_path = os.path.join(logs_basedir, 'cmap.pt')
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
        model.load_state_dict(torch.load(os.path.join(model_basedir, 'weights', 'pointnet_cvae_model.pth')))
        model = model.to(device)
        model.eval()
        object_list = json.load(open(os.path.join(model_basedir, "objects.json"), 'rb'))['test']
    else:
        raise NotImplementedError("Unknown model name")

    cmap = []
    for object_name in object_list:
        print(f'object name: {object_name}')
        object_mesh: tm.Trimesh
        object_mesh = tm.load(os.path.join('data/object', object_name.split('+')[0], object_name.split("+")[1],
                                           f'{object_name.split("+")[1]}.stl'))
        for i_sample in range(args.num_per_object):
            cmap_ood_sample = {'object_name': object_name,
                              'i_sample': i_sample,
                              'object_point_cloud': None,
                              'contact_map_value': None}
            print(f'[{i_sample}/{args.num_per_object}] | {object_name}')
            object_point_cloud, faces_indices = trimesh.sample.sample_surface(mesh=object_mesh, count=2048)
            contact_points_normal = torch.tensor([object_mesh.face_normals[x] for x in faces_indices]).float()
            object_point_cloud = torch.Tensor(object_point_cloud).float()
            object_point_cloud = torch.cat([object_point_cloud, contact_points_normal], dim=1).to(device)
            z_latent_code = torch.randn(1, model.latent_size, device=device).float()
            contact_map_value = model.inference(object_point_cloud[:, :3].unsqueeze(0), z_latent_code).squeeze(0)
            # process the contact map value
            contact_map_value = contact_map_value.detach().cpu().unsqueeze(1)
            contact_map_value = pre_process_contact_map_goal(contact_map_value).to(device)
            contact_map_goal = torch.cat([object_point_cloud, contact_map_value], dim=1)

            cmap_ood_sample['object_point_cloud'] = object_point_cloud
            cmap_ood_sample['contact_map_value'] = contact_map_value
            cmap.append(cmap_ood_sample)
            vis_data = []
            vis_data += [plot_point_cloud_cmap(contact_map_goal[:, :3].cpu().detach().numpy(),
                                               contact_map_goal[:, 6].cpu().detach().numpy())]
            vis_data += [plot_mesh_from_name(f'{object_name}')]
            fig = go.Figure(data=vis_data)
            fig.write_html(os.path.join(vis_cmap_dir, f'{object_name}-{i_sample}.html'))
    torch.save(cmap, cmap_path)
