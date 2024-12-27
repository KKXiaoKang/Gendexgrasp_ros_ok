"""
    CMapAdam 是实际执行优化算法的类，被导入用于在 AdamGrasp 中调用。
    torch 是用于深度学习的 PyTorch 库。
    tqdm 是一个用于显示循环进度条的库，便于监控训练或优化
"""
from utils_model.CMapAdam import CMapAdam
import torch
from tqdm import tqdm


class AdamGrasp:
    """
        AdamGrasp.py 文件定义了一个名为 AdamGrasp 的类（负责梯度下降的优化）
        它实现了基于 CMapAdam 优化算法的抓取生成过程。
        该类封装了多个参数设置、优化步骤执行，以及抓取结果的记录和保存功能

        class members:
            AdamGrasp 是一个用于执行抓取生成任务的类，其构造函数接受多个参数来配置优化过程。
            robot_name：机器人的名称。
            writer：TensorBoard 的写入器，用于记录优化过程中产生的数据。
            contact_map_goal：目标接触图，通常是一个由点云和接触值组成的张量。
            num_particles：粒子的数量，用于粒子群优化。
            init_rand_scale：初始化随机噪声的范围。
            max_iter：最大优化迭代次数。
            steps_per_iter：每次迭代的步数。
            learning_rate：学习率。
            device：计算设备（例如 'cuda' 或 'cpu'）。
            energy_func_name：用于优化的能量函数的名称
    """
    def __init__(self, robot_name, writer, contact_map_goal=None,
                 num_particles=32, init_rand_scale=0.5, max_iter=300, steps_per_iter=2,
                 learning_rate=5e-3, device='cuda', energy_func_name='align_dist'):
        self.writer = writer
        self.robot_name = robot_name
        self.contact_map_goal = contact_map_goal
        self.num_particles = num_particles
        self.init_rand_scale = init_rand_scale
        self.learning_rate = learning_rate
        self.device = device
        self.max_iter = max_iter
        self.steps_per_iter = steps_per_iter
        self.energy_func_name = energy_func_name

        # 初始化 CMapAdam 模型
        """
            (1) 初始化 CMapAdam 类的一个实例 opt_model，它负责执行实际的优化计算
            (2) CMapAdam 使用了类似的参数设置，包括机器人名称、粒子数量、学习率、能量函数名称等
        """
        self.opt_model = CMapAdam(robot_name=robot_name, contact_map_goal=None, num_particles=self.num_particles,
                                  init_rand_scale=init_rand_scale, learning_rate=learning_rate, energy_func_name=self.energy_func_name,
                                  device=device)

    def run_adam(self, object_name, contact_map_goal, running_name):
        """
            run_adam 是该类的核心方法，执行粒子群优化来生成抓取

            object_name：物体的名称。
            contact_map_goal：目标接触图，用于引导抓取优化。
            running_name：当前运行任务的名称，用于标识日志和输出
        """
        # 初始化变量与模型重置
        """
            q_trajectory: 是用来存储优化过程中每个迭代产生的抓取姿态的列表
            self.opt_model.reset: 重置优化模型的状态，使用提供的 contact_map_goal（目标接触图）、running_name（当前运行任务的名称）和 energy_func_name（能量函数名称）来初始化优化过程
        """
        q_trajectory = []
        self.opt_model.reset(contact_map_goal, running_name, self.energy_func_name)

        # 初始抓取姿态的获取
        """
            (1) 通过调用 self.opt_model.get_opt_q() 获取初始的抓取姿态 opt_q
            (2) 由于不需要进行梯度计算，因此使用 torch.no_grad()，提高代码执行效率
            (3) 将获取到的初始抓取姿态添加到 q_trajectory 列表中
        """
        with torch.no_grad(): # 关闭梯度计算
            opt_q = self.opt_model.get_opt_q() # 获取要优化的抓取姿态opt_q
            # print(" opt_q : ", opt_q)
            q_trajectory.append(opt_q.clone().detach()) # 将获取到的初始抓取姿态添加到q_trajectory列表当中
        
        # 优化迭代
        """
            (1) 进入一个循环，该循环执行 self.max_iter 次迭代，每次迭代都调用 self.opt_model.step() 执行一步优化
            (2) 每次迭代结束后，通过 get_opt_q() 获取当前最优的抓取姿态，并将其存储到 q_trajectory 
            (3) 每隔 iters_per_print 次迭代，或者在最后一次迭代时，输出当前最小的能量值和对应的索引
        """
        iters_per_print = self.max_iter // 4
        for i_iter in tqdm(range(self.max_iter), desc=f'{running_name}'):
            self.opt_model.step()
            with torch.no_grad():
                opt_q = self.opt_model.get_opt_q()
                q_trajectory.append(opt_q.clone().detach())
            if i_iter % iters_per_print == 0 or i_iter == self.max_iter - 1:
                print(f'min energy: {self.opt_model.energy.min(dim=0)[0]:.4f}')
                print(f'min energy index: {self.opt_model.energy.min(dim=0)[1]}')
            # 记录与返回结果
            """
                (1) 通过 self.opt_model.energy 获取当前能量值，并将其记录到 tensorboard 中以便可视化
                (2) 将存储的抓取姿态列表 q_trajectory 转换为张量并返回，同时返回对应的能量值和每次迭代的步数
            """
            with torch.no_grad():
                energy = self.opt_model.energy.detach().cpu().tolist()
                tag_scaler_dict = {f'{i_energy}': energy[i_energy] for i_energy in range(len(energy))}
                self.writer.add_scalars(main_tag=f'energy/{running_name}', tag_scalar_dict=tag_scaler_dict, global_step=i_iter)
                self.writer.add_scalar(tag=f'index/{running_name}', scalar_value=energy.index(min(energy)), global_step=i_iter)
        q_trajectory = torch.stack(q_trajectory, dim=0).transpose(0, 1)
        return q_trajectory, self.opt_model.energy.detach().cpu().clone(), self.steps_per_iter
