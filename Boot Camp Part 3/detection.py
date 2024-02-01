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

import sys
from sensor_msgs.msg import PointField  



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
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in  a terminal.
        """

        # Convert ROS ->  NumPy
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
        raw_colors = np.asarray(ds_o3d_point_cloud.colors)
        colors = (raw_colors * 255).astype(np.uint8)

        # output log anout object detection
        red_object_color = (252, 102, 100)
        green_object_color = (0, 120, 110)
        dist_threshold = 1.0  
        red_tolerance = 50
        green_tolerance = 65

        red_color_differences =np.abs( np.linalg.norm(colors - red_object_color, axis=1))
        green_color_differences =np.abs( np.linalg.norm(colors - green_object_color, axis=1))
        
        GREEN = '\033[92m'
        RED = '\033[91m'
        RESET = '\033[0m'
        filtered_points = []

        for i in range(len(colors)):
            dist = np.sqrt(points[i][0]**2 + points[i][1]**2 + points[i][2]**2)

            if red_color_differences[i] < red_tolerance and dist < dist_threshold:
                self.get_logger().info(RED +f"Red Object detected! "+ RESET)
                filtered_points.append([points[i][0], points[i][1], points[i][2], colors[i][0], colors[i][1], colors[i][2]])

            if green_color_differences[i] < green_tolerance and dist < dist_threshold+0.1:
                self.get_logger().info(GREEN+f"Green Object detected! "+ RESET)
                filtered_points.append([points[i][0], points[i][1], points[i][2], colors[i][0], colors[i][1], colors[i][2]])
        
        
        if filtered_points:
            # Create a new PointCloud2 message with filtered data
            filtered_pc2_msg = PointCloud2()
            filtered_pc2_msg.header = msg.header
            filtered_pc2_msg.height = msg.height
            filtered_pc2_msg.width = len(filtered_points)
            filtered_pc2_msg.fields = msg.fields

            filtered_pc2_msg.is_bigendian = False
            filtered_pc2_msg.point_step = msg.point_step  # Length of a point in bytes (4 floats)
            filtered_pc2_msg.row_step = filtered_pc2_msg.point_step * filtered_pc2_msg.width
            filtered_pc2_msg.data = bytes()  # Initialize the data as empty
            filtered_pc2_msg.is_dense = True  # Set to True if there are no invalid points

            # Create an array to hold the binary data for all points
            filtered_data = bytearray()

            for point in filtered_points:
                x, y, z, r, g, b = point

                # Convert x, y, z, and rgb values to bytes and append them to filtered_data
                point_data = bytearray(struct.pack('<ffffI', x, y, z, 0.0, 0))  # Insert four zeros at offset 12
                rgb_value = ((int(r) << 16) | (int(g) << 8) | int(b))
                struct.pack_into('<I', point_data, 16, rgb_value)  # Update the rgb value at offset 16
                filtered_data.extend(point_data)
            
            filtered_pc2_msg.data = bytes(filtered_data)

            # Publish the filtered PointCloud2 message
            self._pub.publish(filtered_pc2_msg)



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