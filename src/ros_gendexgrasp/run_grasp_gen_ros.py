import argparse
import json
import os.path
import time
import sys
import shutil
from utils_model.AdamGrasp import AdamGrasp
import torch
import plotly.graph_objects as go
from utils.visualize_plotly import plot_point_cloud, plot_point_cloud_cmap, plot_mesh_from_name
from utils.set_seed import set_global_seed
from utils.get_models import get_handmodel
from torch.utils.tensorboard import SummaryWriter
import yaml


import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix
from sensor_msgs.msg import JointState
import numpy as np

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

def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot_name', default='lejuhand', type=str)
    parser.add_argument('--cmap_dir', required=True, type=str)
    parser.add_argument('--max_iter', default=100, type=int)
    parser.add_argument('--steps_per_iter', default=1, type=int)
    parser.add_argument('--num_particles', default=32, type=int)
    parser.add_argument('--learning_rate', default=5e-3, type=float)
    parser.add_argument('--init_rand_scale', default=0.5, type=float)
    parser.add_argument('--object_name', default='contactdb+apple', type=str)
    parser.add_argument('--energy_func', default='align_dist', type=str)
    parser.add_argument('--comment', default='default', type=str)
    parser.add_argument('--pose_topic', default='/best_grasp_pose', type=str)
    parser.add_argument('--hand_topic', default='/best_hand_pos', type=str)
    args_ = parser.parse_args()
    tag = str(time.time())
    return args_, tag

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

if __name__ == '__main__':
    set_global_seed(seed=42)
    torch.set_printoptions(precision=4, sci_mode=False, edgeitems=8)
    args, time_tag = get_parser()
    print(args)
    print(f'double check....')
    
    # ros工程化
    rospy.init_node('grasp_pose_publisher')
    pose_pub = rospy.Publisher(args.pose_topic, PoseStamped, queue_size=10)     # 发布/best_grasp_pose 用于发布每个接触图下的最优抓取姿态PoseStamped
    hand_pos_pub = rospy.Publisher(args.hand_topic, JointState, queue_size=10) # 发布/best_hand_pos   用于发布接触图下当前手腕位置PoseStamped

    logs_basedir = os.path.join('logs_gen', f'leju-{args.comment}', f'{args.energy_func}')
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
    # 加载接触图数据集
    # load cmap inference dataset
    """
        尝试加载接触图数据集 (cmap.pt)，这是一个预处理后的数据文件，包含物体点云和接触图值。
    """
    try:
        cmap_dataset = torch.load(os.path.join(args.cmap_dir, f'cmap.pt'))
    except FileNotFoundError:
        raise NotImplementedError('occur when load CMap Dataset...')

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

    # 主处理循环
    """
        (1) 遍历加载的接触图数据集，为每个符合条件的物体和样本生成抓取
        (2) 使用 AdamGrasp 模型的 run_adam 方法执行优化，生成最佳的抓取姿势
    """
    adam_object_name = args.object_name
    for i_data in cmap_dataset:
        object_name = i_data['object_name']
        if object_name != adam_object_name:
            continue
        object_point_cloud = i_data['object_point_cloud']
        i_sample = i_data['i_sample']
        contact_map_value = i_data['contact_map_value']
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

        rospy.sleep(1.0)  # 发布间隔