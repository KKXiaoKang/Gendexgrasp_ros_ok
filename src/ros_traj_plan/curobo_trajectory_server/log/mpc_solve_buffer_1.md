Current goal_buffer: 
Goal(name='goal', 
    goal_state=JointState(
    position=tensor([[0.0000, 1.5700, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]],device='cuda:0'), 
    velocity=tensor([[0., 0., 0., 0., 0., 0., 0.]], device='cuda:0'), 
    acceleration=tensor([[0., 0., 0., 0., 0., 0., 0.]], device='cuda:0'), 
    joint_names=['zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint'], 
    jerk=tensor([[0., 0., 0., 0., 0., 0., 0.]], device='cuda:0'), 
    tensor_args=TensorDeviceType(device=device(type='cuda', index=0), dtype=torch.float32, collision_geometry_dtype=torch.float32, collision_gradient_dtype=torch.float32, collision_distance_dtype=torch.float32), aux_data={}), 
    
    goal_pose=Pose(
        position=tensor([[0.4000, 0.3000, 0.3000]], device='cuda:0'), 
        quaternion=tensor([[1., 0., 0., 0.]], device='cuda:0'), 
        rotation=None, 
        batch=1, 
        n_goalset=1, 
        name='ee_link', 
        normalize_rotation=True), 
        links_goal_pose=None, 
        current_state=JointState(position=tensor([[0.0000, 1.5700, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]],device='cuda:0'), 
        velocity=tensor([[0., 0., 0., 0., 0., 0., 0.]], device='cuda:0'), 
        acceleration=tensor([[0., 0., 0., 0., 0., 0., 0.]], device='cuda:0'), 
        joint_names=['zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint'], 
        jerk=tensor([[0., 0., 0., 0., 0., 0., 0.]], device='cuda:0'), 
        tensor_args=TensorDeviceType(device=device(type='cuda', index=0), 
        dtype=torch.float32, collision_geometry_dtype=torch.float32, 
        collision_gradient_dtype=torch.float32, 
        collision_distance_dtype=torch.float32), 
        aux_data={}), 
        retract_state=None, 
        batch=1, 
        batch_pose_idx=tensor([[0]], 
        device='cuda:0', 
        dtype=torch.int32), 
        batch_goal_state_idx=tensor([[0]], 
        device='cuda:0', 
        dtype=torch.int32), 
        batch_retract_state_idx=tensor([[0]], 
        device='cuda:0', dtype=torch.int32), 
        batch_current_state_idx=tensor([[0]], 
        device='cuda:0', dtype=torch.int32), 
        batch_enable_idx=None, 
        batch_world_idx=tensor([[0]], 
        device='cuda:0', 
        dtype=torch.int32), 
        update_batch_idx_buffers=True, n_goalset=1)
        
[INFO] [1735572947.566341]: Updated MPC goal to Pose(position=tensor([[0.4000, 0.3000, 0.3000]], device='cuda:0'), quaternion=tensor([[1., 0., 0., 0.]], device='cuda:0'), rotation=None, batch=1, n_goalset=1, name='ee_link', normalize_rotation=True)