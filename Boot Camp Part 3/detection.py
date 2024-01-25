#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from open3d import open3d as o3d

import ctypes
import struct

class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)

        # Subscribe to point cloud topic and call callback function on each recieved message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)

    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published 
        on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect 
        objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """

        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        xyz = gen[:,:3]
        rgb = np.empty(xyz.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f' , c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            rgb[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8) 
            rgb[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8) 
            rgb[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        rgb = rgb.astype(np.float32) / 255

        # Convert NumPy -> Open3D
        o3d_point_cloud = o3d.geometry.PointCloud()    
        o3d_point_cloud.points = o3d.utility.Vector3dVector(xyz)
        o3d_point_cloud.colors = o3d.utility.Vector3dVector(rgb)

        # Downsample the point cloud to 5 cm
        ds_o3d_point_cloud = o3d_point_cloud.voxel_down_sample(voxel_size=0.05)

        # Convert Open3D -> NumPy
        points = np.asarray(ds_o3d_point_cloud.points)
        colors = np.asarray(ds_o3d_point_cloud.colors)
        colors = (colors * 255).astype(np.uint8)


        red_object_color = (252, 102, 100)
        green_object_color = (0, 112, 102)
        tolerance = 50


        red_color_differences =np.abs( np.linalg.norm(colors - red_object_color, axis=1))
        green_color_differences =np.abs( np.linalg.norm(colors - green_object_color, axis=1))

        if ((red_color_differences < tolerance).any()):
            self.get_logger().info("Red Object detected!")

        if ((green_color_differences < tolerance).any()):
            self.get_logger().info("Green Object detected!")

        







def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()