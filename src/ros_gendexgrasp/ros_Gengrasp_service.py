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
import re
from ros_gendexgrasp.srv import dex_gengrasp_srv, dex_gengrasp_srvResponse, dex_gengrasp_srvRequest
from ros_gendexgrasp.srv import dex_grasp_index, dex_grasp_indexResponse, dex_grasp_indexRequest

SEED_NUM = 0  # 全局随机种子
STATUS = 0    # 全局状态
GRASP_INDEX = 0  # 全局抓取姿态的索引
FIRST_INIT_FALG = True  # 全局是否第一次初始化

class GenGraspService:
    def __init__(self, args):
        self.args = args
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.logs_basedir = os.path.join('logs_ros_gen', f'leju-{args.comment}', f'{args.energy_func}')
        self.yaml_path = os.path.join(self.logs_basedir, 'config.yaml')
        self.pose_pub = rospy.Publisher(args.pose_topic, PoseStamped, queue_size=10)
        self.hand_pos_pub = rospy.Publisher(args.hand_topic, JointState, queue_size=10)
        self.setup_directories()
        self.writer = SummaryWriter(self.tb_dir)

        # 通过ros服务，借用随机种子，生成抓取姿态
        self.service = rospy.Service(
            '/gendex_grasp_service', dex_gengrasp_srv, self.handle_gendex_grasp_service
        )
        
        # ros服务 | 设置从第几个姿态开始生成 | 获取当前是第几个姿态 
        self.set_grasp_index_service = rospy.Service(
            '/set_grasp_index', dex_grasp_index, self.handle_set_grasp_index_service

        )
        self.get_grasp_index_service = rospy.Service(
            '/get_grasp_index', dex_grasp_index, self.handle_get_grasp_index_service
        )

        # 模型初始化
        self.model = AdamGrasp(robot_name=args.robot_name, writer=self.writer, contact_map_goal=None,
                               num_particles=args.num_particles, init_rand_scale=args.init_rand_scale,
                               max_iter=args.max_iter, steps_per_iter=args.steps_per_iter, learning_rate=args.learning_rate,
                               device=self.device, energy_func_name=args.energy_func)
        self.handmodel = get_handmodel(robot=args.robot_name, batch_size=1, device=self.device)
        self.object_vis_data = plot_mesh_from_name(args.object_name)

    def handle_set_grasp_index_service(self, req):
        global GRASP_INDEX
        GRASP_INDEX = req.set_grasp_index

        rospy.loginfo(f"Set grasp index to {GRASP_INDEX}")
        return dex_grasp_indexResponse(result=True, get_grasp_index=0)

    def handle_get_grasp_index_service(self, req):
        global GRASP_INDEX
        rospy.loginfo(f"Get grasp index: {GRASP_INDEX}")
        return dex_grasp_indexResponse(result=True, get_grasp_index=GRASP_INDEX)

    def handle_gendex_grasp_service(self, req):
        global SEED_NUM
        global STATUS
        
        # 获取请求的时间戳信息
        request_time = req.header.stamp
        request_time_str = f"{request_time.secs}.{request_time.nsecs}"  # 秒和纳秒拼接为字符串

        # 打印请求时间戳信息
        rospy.loginfo(f"Received request at time: {request_time_str}")   
        
        # 读取请求参数 
        SEED_NUM = req.seed_num
        STATUS = req.status

        # 返回结果
        return dex_gengrasp_srvResponse(result=True)

    def setup_directories(self):
        self.tb_dir = os.path.join(self.logs_basedir, 'tb_dir')
        self.tra_dir = os.path.join(self.logs_basedir, 'tra_dir')
        self.vis_dir = os.path.join(self.logs_basedir, 'vis_dir')
        os.makedirs(self.logs_basedir, exist_ok=True)
        os.makedirs(self.tra_dir, exist_ok=True)
        os.makedirs(self.tb_dir, exist_ok=True)
        os.makedirs(self.vis_dir, exist_ok=True)

        f = open(os.path.join(self.logs_basedir, 'command.txt'), 'w')
        f.write(' '.join(sys.argv))
        f.close()

        src_dir_list = ['utils', 'utils_model', 'utils_data']
        os.makedirs(os.path.join(self.logs_basedir, 'src'), exist_ok=True)
        for fn in os.listdir('.'):
            if fn.endswith('.py'):
                shutil.copy(fn, os.path.join(self.logs_basedir, 'src', fn))
        for src_dir in src_dir_list:
            for fn in os.listdir(f'{src_dir}'):
                os.makedirs(os.path.join(self.logs_basedir, 'src', f'{src_dir}'), exist_ok=True)
                if fn.endswith('.py') or fn.endswith('.yaml'):
                    shutil.copy(os.path.join(f'{src_dir}', fn), os.path.join(self.logs_basedir, 'src', f'{src_dir}', fn))

    def save_best_q_to_yaml(self, best_q, object_name, i_sample):
        best_q_list = best_q.cpu().numpy().tolist()
        config_data = {f'{object_name}_{i_sample}_best_q': best_q_list}
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as file:
                existing_data = yaml.safe_load(file) or {}
        else:
            existing_data = {}
        existing_data.update(config_data)
        with open(self.yaml_path, 'w') as file:
            yaml.safe_dump(existing_data, file)

    def best_q_to_posestamped(self, best_q, frame_id="torso"):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = frame_id
        translation = best_q[0, :3].cpu().numpy()
        rotation_matrix_components = best_q[0, 3:9].cpu().numpy()
        rotation_matrix = np.zeros((3, 3))
        rotation_matrix[:, 0] = rotation_matrix_components[:3]
        rotation_matrix[:, 1] = rotation_matrix_components[3:6]
        rotation_matrix[:, 1] -= np.dot(rotation_matrix[:, 0], rotation_matrix[:, 1]) * rotation_matrix[:, 0]
        rotation_matrix[:, 1] /= np.linalg.norm(rotation_matrix[:, 1])
        rotation_matrix[:, 2] = np.cross(rotation_matrix[:, 0], rotation_matrix[:, 1])
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = rotation_matrix
        quaternion = quaternion_from_matrix(homogeneous_matrix)
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        return pose_msg

    def best_q_to_posehand(self, best_q, frame_id="torso"):
        joint_names = [
            "base_link", "thumb1", "thumb2", "index1", "index2",
            "middle1", "middle2", "ring1", "ring2", "little1", "little2"
        ]
        joint_positions = best_q[0, 9:19].cpu().numpy()
        robot_hand_joint_msg = JointState()
        robot_hand_joint_msg.header.stamp = rospy.Time.now()
        robot_hand_joint_msg.header.frame_id = frame_id
        robot_hand_joint_msg.name = joint_names
        robot_hand_joint_msg.position = joint_positions
        return robot_hand_joint_msg

    def check_and_call_service(self, event):
        # 全局随机种子 | 全局状态
        global SEED_NUM, STATUS
        if STATUS:
            rospy.loginfo("开始生成抓取姿态 | Calling grasp start service...")
            set_global_seed(SEED_NUM)
            self.gen_grasp_map()

    # 提取文件名中的数字进行排序
    def extract_number(self, filename):  # 加入 self 参数
        # 使用正则表达式从文件名中提取数字
        numbers = re.findall(r'\d+', filename)
        return int(numbers[-1]) if numbers else -1

    def set_global_grasp_index(self, index):
        global GRASP_INDEX
        GRASP_INDEX = index

    def get_global_grasp_index(self):
        global GRASP_INDEX
        return GRASP_INDEX
    
    def gen_grasp_map(self):
        # 引入全局状态
        global STATUS
        global GRASP_INDEX
        global FIRST_INIT_FALG

        cmap_files = sorted(glob.glob(os.path.join(self.args.cmap_dir, 'contactdb+water_bottle_sample_*.pt')), key=self.extract_number)  # 调用 self.extract_number
        print("cmap_files:", cmap_files)

        if GRASP_INDEX >= len(cmap_files):
            print(f"GRASP_INDEX ({GRASP_INDEX}) exceeds the number of files ({len(cmap_files)}). Resetting to 0.")
            GRASP_INDEX = 0  # 如果索引超出文件数量，重置为0
        
        # 判断是否为全局第一次初始化
        if FIRST_INIT_FALG == True:
            GRASP_INDEX = GRASP_INDEX
        elif FIRST_INIT_FALG == False:
            GRASP_INDEX = GRASP_INDEX + 1

        for idx, cmap_file in enumerate(cmap_files[GRASP_INDEX:], start=GRASP_INDEX):
            # first init过后就会把标志位设置为False
            FIRST_INIT_FALG = False
            
            # 如果全局状态为0，则退出抓取姿态生成
            if STATUS == 0:
                print( " 接收到grasp服务停止调用的信息，正在退出......")
                break
            # 处理手部姿态 | 索引+1
            GRASP_INDEX = idx
            self.set_global_grasp_index(GRASP_INDEX)
            print( " GRASP_INDEX : ", self.get_global_grasp_index())

            print(f"Processing {cmap_file}...")
            try:
                cmap_dataset = torch.load(cmap_file)
            except FileNotFoundError:
                raise NotImplementedError(f'Could not load {cmap_file}')
            object_name = self.args.object_name
            object_point_cloud = cmap_dataset['object_point_cloud']
            i_sample = cmap_dataset['i_sample']
            contact_map_value = cmap_dataset['contact_map_value']
            running_name = f'{object_name}+{i_sample}'
            contact_map_goal = torch.cat([object_point_cloud, contact_map_value], dim=1).to(self.device)
            record = self.model.run_adam(object_name=object_name, contact_map_goal=contact_map_goal, running_name=running_name)
            with torch.no_grad():
                q_tra, energy, steps_per_iter = record
                i_record = {'q_tra': q_tra[:, -1:, :].detach(),
                            'energy': energy,
                            'steps_per_iter': steps_per_iter,
                            'comment': self.args.comment,
                            'object_name': object_name,
                            'i_sample': i_sample}
                torch.save(i_record, os.path.join(self.tra_dir, f'tra-{object_name}-{i_sample}.pt'))
                best_q = q_tra[energy.argmin(), -1, :].unsqueeze(0)
                # 将best_q保存到yaml文件中
                self.save_best_q_to_yaml(best_q, object_name, i_sample)
                # html可视化
                self.handmodel.update_kinematics(q=best_q)
                vis_data = self.handmodel.get_plotly_data(color='pink', opacity=1.0)
                vis_data += [self.object_vis_data]
                fig = go.Figure(data=vis_data)
                fig.write_html(os.path.join(self.vis_dir, f'grasppose-{object_name}-{i_sample}.html'))
                # 发布抓取姿态
                pose_msg = self.best_q_to_posestamped(best_q)
                self.pose_pub.publish(pose_msg)
                rospy.loginfo(f"Published best grasp pose for {object_name}-{i_sample}")
                # 发布灵巧手的手指数据
                hand_msg = self.best_q_to_posehand(best_q)
                self.hand_pos_pub.publish(hand_msg)
                rospy.loginfo(f"Published best grasp hand_msg 灵巧手 for {object_name}-{i_sample}")

def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--comment', default='leju', type=str)
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

if __name__ == "__main__":

    rospy.init_node('gendex_grasp_service_node')

    args, tag = get_parser()

    grasp_service = GenGraspService(args)

    rospy.Timer(rospy.Duration(0.1), grasp_service.check_and_call_service) # 定时器用于检索姿态生成器状态 是否开始生成抓取姿态

    rospy.spin()