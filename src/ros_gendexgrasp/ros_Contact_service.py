"""
    工厂模式GenDexGrasp的ROS服务端模块

    contract模块： 根据物体的点云自行选择规则然后生成接触图
    指定种子生成接触图 
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

from ros_gendexgrasp.srv import dex_contract_srv, dex_contract_srvResponse, dex_contract_srvRequest

class GendexContact:
    """
        GendexContact接触图生成类
        根据/gendex_contact_service节点的请求，生成接触图

        int32 contact_num
        uint8 seed_num
        ---
        bool result 
    """
    def __init__(self, args):
        self.args = args
        self.device = "cuda"
        self.logs_basedir = os.path.join('logs_ros_inf_cvae', f'{args.s_model}', f'{args.pre_process}', f'{args.comment}')
        self.vis_cmap_dir = os.path.join(self.logs_basedir, 'vis_cmap_dir')
        self.cmap_path = os.path.join(self.logs_basedir, 'cmap.pt')

        # 通过ros服务，借用随机种子作为参考生成固定数量的接触图
        self.service = rospy.Service(
            '/gendex_contact_service', dex_contract_srv, self.handle_gendex_contact_service
        )

        os.makedirs(self.logs_basedir, exist_ok=True)
        os.makedirs(self.vis_cmap_dir, exist_ok=True)
        
        # 打开特定面片选取开关 
        self.open_spec_flag = True

        if args.s_model == 'PointNetCVAE_SqrtFullRobots':
            self.model = self._load_pointnet_model()
        else:
            raise NotImplementedError("Unknown model name")

        self.pre_process_map = {'sharp_lift': self.pre_process_sharp_clamp, 'identity': self.identity_map}

    def handle_gendex_contact_service(self, req):
        # 获取请求的时间戳信息
        request_time = req.header.stamp
        request_time_str = f"{request_time.secs}.{request_time.nsecs}"  # 秒和纳秒拼接为字符串

        # 打印请求时间戳信息
        rospy.loginfo(f"Received request at time: {request_time_str}")

        # 获取指定的参数
        seed_num = req.seed_num
        contact_num = req.contact_num

        # 打印请求参数
        rospy.loginfo(f"Request parameters - Seed: {seed_num}, Contact num: {contact_num}")

        # 设置全局随机种子
        set_global_seed(seed_num)

        # 生成指定数量的接触图
        self.gen_contact_map(contact_num)

        # 返回结果
        return dex_contract_srvResponse(result=True)

    def _load_pointnet_model(self):
        model_basedir = 'ckpts/SqrtFullRobots'
        from ckpts.SqrtFullRobots.src.utils_model.PointNetCVAE import PointNetCVAE
        model = PointNetCVAE(latent_size=128,
                             encoder_layers_size=[4, 64, 128, 512],
                             decoder_global_feat_size=512,
                             decoder_pointwise_layers_size=[3, 64, 64],
                             decoder_global_layers_size=[64, 128, 512],
                             decoder_decoder_layers_size=[64 + 512 + 128, 512, 64, 64, 1])
        model.load_state_dict(torch.load(os.path.join(model_basedir, 'weights', 'pointnet_cvae_model.pth')))
        model = model.to(self.device)
        model.eval()
        return model

    def pre_process_sharp_clamp(self, contact_map):
        gap_th = 0.5
        gap_th = min(contact_map.max().item(), gap_th)
        delta_th = (1 - gap_th)
        contact_map[contact_map > 0.4] += delta_th
        contact_map = torch.clamp_max(contact_map, 1.)
        return contact_map

    def identity_map(self, contact_map):
        return contact_map

    def gen_contact_map(self, contact_num):
        pre_process_contact_map_goal = self.pre_process_map[self.args.pre_process]
        object_list = json.load(open(os.path.join('ckpts/SqrtFullRobots', "objects.json"), 'rb'))['test']
        np.set_printoptions(threshold=np.inf) # 设置打印时输出完整数组，不省略
        for object_name in object_list:
            object_mesh = tm.load(os.path.join('data/object', object_name.split('+')[0], object_name.split("+")[1],
                                               f'{object_name.split("+")[1]}.stl'))
            count_contact_map = 0

            while not rospy.is_shutdown() and count_contact_map < contact_num:
                cmap_ood_sample = {'object_name': object_name, 'i_sample': count_contact_map, 'object_point_cloud': None, 'contact_map_value': None}

                # 随机采样整个网格的面片索引2048个
                object_point_cloud, faces_indices = trimesh.sample.sample_surface(mesh=object_mesh, count=2048)
                # 进行排序
                sorted_faces_indices = np.sort(faces_indices)
                # print( " ------------- sorted_faces_indices --------------- : ", sorted_faces_indices)
                # 计算排序之后的点的法向量
                contact_points_normal = torch.tensor([object_mesh.face_normals[x] for x in sorted_faces_indices]).float()
                # 将点云数据转换为 PyTorch Tensor
                object_point_cloud = torch.Tensor(object_point_cloud).float()

                # 判断是否开启特定样片采样
                if self.open_spec_flag: # TODO:检索算法准确性
                    print(" --------------- faces_indices -------------------------- : ", len(faces_indices))
                    specified_faces_indices = np.concatenate([
                        sorted_faces_indices[0:1120],    # 第 100 到 200 个
                        sorted_faces_indices[1700:2049]  # 第 1600 到 2049 个
                    ])
                    # specified_faces_indices = sorted_faces_indices[0:2049] # 全采样
                    # specified_faces_indices = sorted_faces_indices[1600:2049] # 部分采样
                    # print(" --------------- specified_faces_indices -------------------------- : ", specified_faces_indices)
                    mask = np.isin(faces_indices, specified_faces_indices)
                    # print(" --------------- mask -------------------------- : ", mask)
                    filtered_object_point_cloud = object_point_cloud[mask]
                    filtered_contact_points_normal = contact_points_normal[mask]
                    object_point_cloud = torch.cat([filtered_object_point_cloud, filtered_contact_points_normal], dim=1).to(self.device)                   
                else: # 不开启特定样片采样，进行全采样
                    object_point_cloud = torch.cat([object_point_cloud, contact_points_normal], dim=1).to(self.device)
                
                # 生成潜在代码 | 使用标准正态分布随机生成一个潜在代码z_latent_code，它将被输入到解码器中用于生成接触图
                z_latent_code = torch.randn(1, self.model.latent_size, device=self.device).float()
                #  调用PointNetCVAE模型的inference方法，将采样的点云坐标（去掉法向量部分）和潜在代码输入模型，输出接触图值contact_map_value
                contact_map_value = self.model.inference(object_point_cloud[:, :3].unsqueeze(0), z_latent_code).squeeze(0)
                # 将接触图值转移到CPU上，并进行必要的后处理（如clamp操作），以确保数值在合适的范围内。
                contact_map_value = contact_map_value.detach().cpu().unsqueeze(1)
                contact_map_value = pre_process_contact_map_goal(contact_map_value).to(self.device)
                # 将经过后处理的接触图值与原始点云数据再次拼接，以形成一个完整的数据结构，包含点的坐标、法向量和接触图值
                contact_map_goal = torch.cat([object_point_cloud, contact_map_value], dim=1)
                # 保存数据与可视化
                cmap_ood_sample['object_point_cloud'] = object_point_cloud
                cmap_ood_sample['contact_map_value'] = contact_map_value

                self._save_contact_map(cmap_ood_sample, object_name, count_contact_map)
                self._visualize_contact_map(contact_map_goal, object_name, count_contact_map)

                count_contact_map += 1

    def _save_contact_map(self, cmap_ood_sample, object_name, i_sample):
        cmap_path = os.path.join(self.logs_basedir, object_name, f'{object_name}_sample_{i_sample}.pt')
        os.makedirs(os.path.dirname(cmap_path), exist_ok=True)
        torch.save(cmap_ood_sample, cmap_path)
        print(f'Saved cmap for {object_name}, sample {i_sample} to {cmap_path}')

    def _visualize_contact_map(self, contact_map_goal, object_name, i_sample):
        """
            (1) 可视化接触图的点云，并生成HTML文件
            (2) 确保路径存在，保存HTML文件
        """
        vis_data = []
        vis_data += [plot_point_cloud_cmap(contact_map_goal[:, :3].cpu().detach().numpy(), contact_map_goal[:, 6].cpu().detach().numpy())]
        vis_data += [plot_mesh_from_name(f'{object_name}')]
        fig = go.Figure(data=vis_data)
    
        # 保存文件前，确保路径存在
        html_output_path = os.path.join(self.vis_cmap_dir, f'{object_name}-{i_sample}.html')
        os.makedirs(os.path.dirname(html_output_path), exist_ok=True)
        
        fig.write_html(html_output_path)
        print(f'Saved visualization for {object_name}, sample {i_sample} to {html_output_path}')


def get_parser():
    parser = argparse.ArgumentParser()
    # 备注信息
    parser.add_argument('--comment', default='leju', type=str) # 备注信息
    # Contract 模块
    parser.add_argument('--pre_process', default='sharp_lift', type=str)              # 指定预处理方法
    parser.add_argument('--s_model', default='PointNetCVAE_SqrtFullRobots', type=str) # 模型名称
    parser.add_argument('--seed', default='42', type=int)        # 随机种子
    parser.add_argument('--num_per_object', default=20, type=int) # 每个对象的样本数
    
    # 解析完后返回参数和当前时间戳
    args_ = parser.parse_args()
    tag = str(time.time())
    return args_, tag

if __name__ == "__main__":
    
    rospy.init_node('gendex_contact_service_node')

    args, tag = get_parser()

    gendex_contract = GendexContact(args)
    
    rospy.spin()