#!/usr/bin/env python

from telnetlib import theNULL
import rclpy 
import math
from rclpy.node import Node

from robp_boot_camp_interfaces.msg import ADConverter
from geometry_msgs.msg import Twist


class WallFollowingController(Node):

    def __init__(self):
        super().__init__('wall_following_controller')

        self.alpha = -0.03
        self.linear_velocity = 1.0  
        
        self.subscription = self.create_subscription(
            ADConverter,'/kobuki/adc', self.distance_sensor_callback,10  
        )

        self.publisher = self.create_publisher(
            Twist,'/motor_controller/twist', 10  
        )

        
    def distance_sensor_callback(self, msg):
        distance_sensor1 = msg.ch1
        distance_sensor2 = msg.ch2

        angular_velocity = self.alpha * (distance_sensor1 - distance_sensor2)

        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = angular_velocity
        self.publisher.publish(twist_msg)
        

            
        


def main():
    rclpy.init()
    node = WallFollowingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
