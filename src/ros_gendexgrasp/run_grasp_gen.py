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

    # parser.add_argument('--dataset', default='SqrtFullRobots', type=str)
    # parser.add_argument('--dataset_id', default='SharpClamp_A3', type=str)
    parser.add_argument('--cmap_dir', required=True, type=str)
    parser.add_argument('--max_iter', default=100, type=int)
    parser.add_argument('--steps_per_iter', default=1, type=int)
    parser.add_argument('--num_particles', default=32, type=int)
    parser.add_argument('--learning_rate', default=5e-3, type=float)
    parser.add_argument('--init_rand_scale', default=0.5, type=float)

    parser.add_argument('--object_name', default='contactdb+apple', type=str)
    parser.add_argument('--energy_func', default='align_dist', type=str)
    parser.add_argument('--comment', default='default', type=str)

    args_ = parser.parse_args()
    tag = str(time.time())
    return args_, tag


if __name__ == '__main__':
    set_global_seed(seed=42)
    torch.set_printoptions(precision=4, sci_mode=False, edgeitems=8)
    args, time_tag = get_parser()
    print(args)
    print(f'double check....')

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
        # 保存抓取结果并进行可视化
        """
            (1) 在优化完成后，将最佳抓取姿势和相关信息保存到文件中
            (2) 更新手部模型的运动学参数以匹配生成的抓取姿势，并使用 Plotly 生成可视化图像
            (3) 将抓取姿势的可视化结果保存为 HTML 文件
        """
        with torch.no_grad():
            q_tra, energy, steps_per_iter = record
            i_record = {'q_tra': q_tra[:, -1:, :].detach(),
                        'energy': energy,
                        'steps_per_iter': steps_per_iter,
                        'comment': args.comment,
                        'object_name': object_name,
                        'i_sample': i_sample}
            torch.save(i_record, os.path.join(tra_dir, f'tra-{object_name}-{i_sample}.pt'))

            # save best grasp pose
            best_q = q_tra[energy.argmin(), -1, :]
            #print("best_q  energy.argmin() : ", best_q)
            best_q = best_q.unsqueeze(0)
            #print("best_q  unsqueeze : ", best_q)
            save_best_q_to_yaml(best_q, object_name, i_sample, yaml_path)
            handmodel.update_kinematics(q=best_q)
            vis_data = handmodel.get_plotly_data(color='pink', opacity=1.0)
            vis_data += [object_vis_data]
            fig = go.Figure(data=vis_data)
            fig.write_html(os.path.join(vis_dir, f'grasppose-{object_name}-{i_sample}.html'))
