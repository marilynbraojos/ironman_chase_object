# Marilyn Braojos 
# Mariam Misabishvili

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point

import cv2
from cv_bridge import CvBridge
import numpy as np
import sys


class VelocityController(Node):
     def __init__(self):        
        super().__init__('velocity_publisher')

        self._pixel_subscriber = self.create_subscription(
            Point,
            'detected_pixel',
            self.pixel_callback, 
            10)
        self._pixel_subscriber 
        
        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 10)


        self.Kp = 0.005  proportional gain 
        self.max_angular_speed = 1.0 

     def pixel_callback(self, msg: Point):
        pix_error = msg.x - msg.y
        twist = Twist()
        twist.linear.x = 0.0

        twist.angular.z = self.Kp * pix_error

        twist.angular.z = max(min(twist.angular.z, self.max_angular_speed), -self.max_angular_speed)


        # if msg.x < msg.y - 50:
        #     twist.angular.z = 0.5
        # elif msg.x > msg.y + 50:
        #     twist.angular.z = -0.5
        # else:
        #     twist.angular.z = 0.0
            
        self._vel_publish.publish(twist)

def main():
    rclpy.init()
    velocity_publisher = VelocityController()

    while rclpy.ok():
        rclpy.spin_once(velocity_publisher)
    
    velocity_publisher.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()