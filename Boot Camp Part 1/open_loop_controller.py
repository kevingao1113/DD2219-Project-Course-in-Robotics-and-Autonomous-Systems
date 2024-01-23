#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles


class OpenLoopController(Node):

    def __init__(self):
        super().__init__('open_loop_controller')
        
        self.publisher_ = self.create_publisher(
            DutyCycles, '/motor/duty_cycles', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        msg = DutyCycles()
        msg.duty_cycle_left = 0.23
        msg.duty_cycle_right = 0.2
        self.publisher_.publish(msg)
        
        self.i += 1


def main():
    rclpy.init()
    node = OpenLoopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()