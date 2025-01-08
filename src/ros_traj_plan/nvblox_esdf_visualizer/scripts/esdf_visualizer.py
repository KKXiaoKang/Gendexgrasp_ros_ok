#!/usr/bin/env python3
# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Licensed under the Apache License, Version 2.0

import rospy
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from nvblox_msgs.srv import EsdfAndGradients, EsdfAndGradientsRequest, EsdfAndGradientsResponse
import numpy as np
import torch
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.sdf.world import WorldCollisionConfig
from curobo.geom.sdf.world_voxel import WorldVoxelCollision
from curobo.geom.types import Cuboid as CuCuboid
from curobo.geom.types import VoxelGrid as CuVoxelGrid
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType


class ESDFVisualizer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('esdf_visualizer')

        # Parameters
        self.voxel_dims = rospy.get_param('~voxel_dims', [2.0, 2.0, 2.0])
        self.grid_position = rospy.get_param('~grid_position', [0.0, 0.0, 0.0])
        self.voxel_size = rospy.get_param('~voxel_size',0.02)
        self.publish_voxel_size = rospy.get_param('~publish_voxel_size', 0.01)
        self.max_publish_voxels = rospy.get_param('~max_publish_voxels', 50000)
        self.esdf_service_name = rospy.get_param('~esdf_service_name', '/nvblox_node/get_esdf_and_gradient')
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'base_link')
        # Voxel publisher
        self.voxel_pub = rospy.Publisher('/curobo/voxels', Marker, queue_size=10)

        # ESDF service client
        rospy.wait_for_service(self.esdf_service_name)
        self.esdf_client = rospy.ServiceProxy(self.esdf_service_name, EsdfAndGradients)
        self.__esdf_req = EsdfAndGradientsRequest()

        # WorldVoxelCollision initialization
        world_cfg = WorldConfig.from_dict({
            'voxel': {
                'world_voxel': {
                    'dims': self.voxel_dims,
                    'pose': [
                        self.grid_position[0],
                        self.grid_position[1],
                        self.grid_position[2],
                        1, 0, 0, 0
                    ],  # x, y, z, qw, qx, qy, qz
                    'voxel_size': self.voxel_size,
                    'feature_dtype': torch.float32,
                },
            },
        })
        tensor_args = TensorDeviceType()
        world_collision_config = WorldCollisionConfig.load_from_dict({
            'checker_type': CollisionCheckerType.VOXEL,
            'max_distance': 10.0,
            'n_envs': 1,
        }, world_cfg, tensor_args)
        self.world_collision = WorldVoxelCollision(world_collision_config)
        self.tensor_args = tensor_args

        # Timer
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
    
    def timer_callback(self, event):
        rospy.loginfo("Calling ESDF service")
        # Prepare request
        aabb_min = Point(
            x=(-0.5 * self.voxel_dims[0]) + self.grid_position[0],
            y=(-0.5 * self.voxel_dims[1]) + self.grid_position[1],
            z=(-0.5 * self.voxel_dims[2]) + self.grid_position[2],
        )
        voxel_dims = Vector3(
            x=self.voxel_dims[0],
            y=self.voxel_dims[1],
            z=self.voxel_dims[2],
        )
        try:
            response = self.send_request(aabb_min, voxel_dims)
            self.fill_marker(response)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def send_request(self, aabb_min_m, aabb_size_m):
        self.__esdf_req.aabb_min_m = aabb_min_m
        self.__esdf_req.aabb_size_m = aabb_size_m
        
        try:
            # 使用同步调用而不是异步调用
            esdf_response = self.esdf_client(self.__esdf_req)
            return esdf_response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
        
    def get_esdf_voxel_grid(self, esdf_data):
        esdf_array = esdf_data.esdf_and_gradients
        array_shape = [
            esdf_array.layout.dim[0].size,
            esdf_array.layout.dim[1].size,
            esdf_array.layout.dim[2].size,
        ]
        array_data = np.array(esdf_array.data)
        array_data = self.tensor_args.to_device(array_data)
        array_data = array_data.reshape(array_shape[0], array_shape[1], array_shape[2]).contiguous()
        array_data = array_data.reshape(-1, 1)
        array_data = -1 * array_data
        array_data[array_data >= 1000.0] = -1000.0
        array_data = array_data + 0.5 * self.voxel_size

        esdf_grid = CuVoxelGrid(
            name='world_voxel',
            dims=self.voxel_dims,
            pose=[
                self.grid_position[0],
                self.grid_position[1],
                self.grid_position[2],
                1, 0.0, 0.0, 0
            ],
            voxel_size=self.voxel_size,
            feature_dtype=torch.float32,
            feature_tensor=array_data,
        )
        return esdf_grid

    def fill_marker(self, esdf_data):
        esdf_grid = self.get_esdf_voxel_grid(esdf_data)
        self.world_collision.update_voxel_data(esdf_grid)
        vox_size = self.publish_voxel_size
        voxels = self.world_collision.get_esdf_in_bounding_box(
            CuCuboid(
                name='test',
                pose=[0.0, 0.0, 0.0, 1, 0, 0, 0],  # x, y, z, qw, qx, qy, qz
                dims=self.voxel_dims,
            ),
            voxel_size=vox_size,
        )
        xyzr_tensor = voxels.xyzr_tensor.clone()
        xyzr_tensor[..., 3] = voxels.feature_tensor
        self.publish_voxels(xyzr_tensor)

    def publish_voxels(self, voxels):
        vox_size = 0.25 * self.publish_voxel_size
        marker = Marker()
        marker.header.frame_id = self.robot_base_frame
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.ns = 'curobo_world'
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(1000.0)
        marker.scale.x = vox_size
        marker.scale.y = vox_size
        marker.scale.z = vox_size
        voxels = voxels[voxels[:, 3] >= 0.0]
        vox = voxels.view(-1, 4).cpu().numpy()
        for i in range(min(len(vox), self.max_publish_voxels)):
            pt = Point(x=float(vox[i, 0]), y=float(vox[i, 1]), z=float(vox[i, 2]))
            d = vox[i, 3]
            rgba = [min(1.0, 1.0 - float(d)), 0.0, 0.0, 1.0]
            color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            marker.points.append(pt)
            marker.colors.append(color)
        marker.header.stamp = rospy.Time.now()
        self.voxel_pub.publish(marker)


if __name__ == '__main__':
    try:
        esdf_visualizer = ESDFVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
