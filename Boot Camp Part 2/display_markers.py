#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from aruco_msgs.msg import MarkerArray
import numpy as np


class DisplayMarkers(Node) :

    def __init__(self) :
        super().__init__('display_markers')


        # Initialize the transform listerner and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to aruco marker topic and call callback function on each recieved message
        self.subscription = self.create_subscription(
                MarkerArray,
                '/aruco/markers',
                self.marker_callback,
                10
            )
        

    def marker_callback(self, msg: MarkerArray) :

        # look up transform from map_frame to marker frame and publish the 
        # position of each marker
        t = None

        for marker in msg.markers:
            try:
                t = self.tf_buffer.lookup_transform(
                    'camera_color_optical_frame',
                    'map',
                    rclpy.time.Time())
                
            except tf2_ros.LookupException as e:
                self.get_logger().warn(f"TF lookup error: {e}")

            tmsg = TransformStamped()
            tmsg.header.stamp = self.get_clock().now().to_msg()
            tmsg.header.frame_id = 'base_link'
            tmsg.child_frame_id  = f'aruco/detected{marker.id}'

            tmsg.transform.translation.x =  marker.pose.pose.position.z + 0.08987
            tmsg.transform.translation.y = -marker.pose.pose.position.x + 0.0175
            tmsg.transform.translation.z = -marker.pose.pose.position.y + 0.10456

            tmsg.transform.rotation.x = 0 *marker.pose.pose.orientation.x
            tmsg.transform.rotation.y = 0 *marker.pose.pose.orientation.y
            tmsg.transform.rotation.z = 0 *marker.pose.pose.orientation.z + 1
            tmsg.transform.rotation.w = 0 *marker.pose.pose.orientation.w

            '''   
            Quaternion rotate by theta degree around certain asix:
            Quaternion = x + y + z + w 
                       = cos(theta) + sin(theta)(c1 *i + c2 *j + c3 *k) 
            (where ratation asix is determined by coeficiens of i,j,k)

            Useful mannual about Quaternion:
            https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html
            https://eater.net/quaternions/video/intro
             '''


            self.tf_broadcaster.sendTransform(tmsg)
      

 
def main() :
    rclpy.init()
    node = DisplayMarkers()
    try :
        rclpy.spin(node)
    except KeyboardInterrupt :
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
