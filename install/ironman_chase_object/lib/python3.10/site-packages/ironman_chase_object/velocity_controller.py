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


        self.Kp = 0.005  # proportional gain 
        self.max_angular_speed = 1.0 
        self.dead_zone = 10  # Pixels within which we don't rotate


        self.last_msg_time = self.get_clock().now()  # Track last received message
        self.timeout_duration = 1.0  # Time (in seconds) before stopping motion
        self.timer = self.create_timer(0.5, self.check_timeout)


    def pixel_callback(self, msg: Point):
        self.last_msg_time = self.get_clock().now()  # Update last received message time

        pix_error = msg.x - msg.y
        twist = Twist()
        twist.linear.x = 0.0

        if abs(pix_error) > self.dead_zone:  # ignore small errors
            twist.angular.z = -self.Kp * pix_error
            twist.angular.z = max(min(twist.angular.z, self.max_angular_speed), -self.max_angular_speed)
        else:
            twist.angular.z = 0.0  # stop rotation if error is small
            
        self._vel_publish.publish(twist)
        
    def check_timeout(self): 
        """ Stop rotation if no new message is received for `timeout_duration` seconds. """
        time_since_last_msg = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9  # Convert to seconds
        if time_since_last_msg > self.timeout_duration:
            twist = Twist()
            twist.angular.z = 0.0  # Stop rotation
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