MpcSolver:
[INFO] [1736243752.146662]: ESDF grid max: 2.009999990463257, min: -999.989990234375
[INFO] [1736243752.147306]: ESDF grid shape: torch.Size([1030301, 1])
 ---- ok 调用到这里 ------- 
Updating voxel data for environment index 0
Voxel Name: world_voxel
Voxel Dimensions: [2.0, 2.0, 2.0]
Voxel Pose: [0.0, 0.0, 0.0, 1, 0.0, 0.0, 0]
Voxel Size: 0.02
Voxel Feature Tensor Shape: torch.Size([1030301, 1])
Voxel Index (obs_idx): 0
Updated _voxel_tensor_list[3] with shape: torch.Size([1, 1, 1030301, 1])
Updated _voxel_tensor_list[0][:3]: tensor([[[2.0000, 2.0000, 2.0000, 0.0200]]], device='cuda:0')
Updated _voxel_tensor_list[0][3]: 0.019999999552965164
Updated _voxel_tensor_list[1][:7]: tensor([-0., -0., -0., 1., -0., -0., -0.], device='cuda:0')
Updated _voxel_tensor_list[2]: 1
[INFO] [1736243752.151533]: Updated ESDF grid
[INFO] [1736243752.152134]: Calling 发布体素世界 world_update_status = self.publish_voxels(xyzr_tensor) function.
[INFO] [1736243752.282551]: Voxels stats: max=2.0256242752075195, min=0.0
[INFO] [1736243752.283554]: Voxels shape: torch.Size([436619, 4])

MotionGen:
[INFO] [1736243838.555019]: ESDF grid max: 2.009999990463257, min: -999.989990234375
[INFO] [1736243838.555426]: ESDF grid shape: torch.Size([1030301, 1])
 ---- ok 调用到这里 ------- 
Updating voxel data for environment index 0
Voxel Name: world_voxel
Voxel Dimensions: [2.0, 2.0, 2.0]
Voxel Pose: [0.0, 0.0, 0.0, 1, 0.0, 0.0, 0]
Voxel Size: 0.02
Voxel Feature Tensor Shape: torch.Size([1030301, 1])
Voxel Index (obs_idx): 0
Updated _voxel_tensor_list[3] with shape: torch.Size([1, 1, 1030301, 1])
Updated _voxel_tensor_list[0][:3]: tensor([[[2.0000, 2.0000, 2.0000, 0.0200]]], device='cuda:0')
Updated _voxel_tensor_list[0][3]: 0.019999999552965164
Updated _voxel_tensor_list[1][:7]: tensor([-0., -0., -0., 1., -0., -0., -0.], device='cuda:0')
Updated _voxel_tensor_list[2]: 1
[INFO] [1736243838.557397]: Updated ESDF grid
[INFO] [1736243838.557916]: Calling 发布体素世界 world_update_status = self.publish_voxels(xyzr_tensor) function.
[INFO] [1736243838.683368]: Voxels stats: max=2.0256242752075195, min=0.020009156316518784
[INFO] [1736243838.684262]: Voxels shape: torch.Size([78726, 4])