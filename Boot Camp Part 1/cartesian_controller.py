#!/usr/bin/env python

import rclpy,math
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles, Encoders
from geometry_msgs.msg import Twist


class CartesianController(Node):

    def __init__(self):
        super().__init__('cartesian_controller')
        
        self.b = 0.23 
        self.r = 0.0352
        # Initialize variables for PI controller
        self.integral_error = [0.0, 0.0]  # Initialize integral error for each wheel

        self.alphal = 0.1
        self.alphar = 0.07

        self.dt = 0.1  # Time difference between iterations (adjust as needed)

        self.real_angular_velocity_left = 0.0
        self.real_angular_velocity_right = 0.0

        self.subscription1 = self.create_subscription(
            Twist,'/motor_controller/twist',self.twist_callback,10)
        self.subscription2 = self.create_subscription(
            Encoders, '/motor/encoders', self.encoders_callback, 10)
        
        self.publisher = self.create_publisher(
            DutyCycles, '/motor/duty_cycles', 10)
        

    def twist_callback(self, msg):
       
        v = msg.linear.x
        w = msg.angular.z

        # Calculate the desired angular velocities for each wheel
        
        desired_angular_velocity_left = (v - w * (self.b))/self.r
        desired_angular_velocity_right = (v + w * (self.b))/self.r

        # Implement P controller for each wheel
        error_left = desired_angular_velocity_left - self.real_angular_velocity_left
        #self.integral_error[i] += error * self.dt
        pwml = self.alphal * error_left #+ self.beta * self.integral_error[i]
        pwml = max(-1.0, min(1.0, pwml))

        error_right = desired_angular_velocity_right - self.real_angular_velocity_right
        #self.integral_error[i] += error * self.dt
        pwmr = self.alphar * error_right #+ self.beta * self.integral_error[i]
        pwmr = max(-1.0, min(1.0, pwmr))

            # Populate DutyCycles message and publish
        duty_cycles_msg = DutyCycles()
            
        
        duty_cycles_msg.duty_cycle_left = pwml
        duty_cycles_msg.duty_cycle_right = pwmr
                
        self.publisher.publish(duty_cycles_msg)



    def encoders_callback(self, msg):
        # Implement logic to handle the Encoders message
        self.real_angular_velocity_left = msg.delta_encoder_left*10/360*2*math.pi
        self.real_angular_velocity_right = msg.delta_encoder_right*10/360*2*math.pi
        # Extract encoder feedback signals and use them for control

def main():
    rclpy.init()
    node = CartesianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
