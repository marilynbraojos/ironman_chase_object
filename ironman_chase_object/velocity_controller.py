#!/usr/bin/env python3
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

class PIDController:
    def __init__(self, Kp, Ki, Kd, clock):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.clock = clock
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = self.clock.now()
    
    def compute(self, error):
        current_time = self.clock.now()
        # Compute time difference in seconds.
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            dt = 1e-16  # Prevent division by zero.

        # Proportional term.
        P = self.Kp * error

        # Integral term.
        self.error_sum += error * dt
        I = self.Ki * self.error_sum

        # Derivative term.
        d_error = (error - self.last_error) / dt
        D = self.Kd * d_error

        self.last_error = error
        self.last_time = current_time

        return P + I + D

class VelocityController(Node):
    def __init__(self):        
        super().__init__('velocity_publisher')

        # Subscriber for detected pixel values (for angular control).
        self._pixel_subscriber = self.create_subscription(
            Point,
            'detected_pixel',
            self.pixel_callback, 
            10)

        # Subscriber for object range (for linear control).
        self._range_subscriber = self.create_subscription(
            Point,
            'object_range',
            self.range_callback, 
            10)
        
        # Publisher for velocity commands.
        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create two separate PID controllers.
        # Angular PID: uses pixel error (desired center pixel minus detected pixel).
        self.angular_pid = PIDController(Kp=0.005, Ki=0.0001, Kd=0.001, clock=self.get_clock())
        # Linear PID: uses distance error (desired distance minus measured distance).
        self.linear_pid = PIDController(Kp=0.1, Ki=0.01, Kd=0.05, clock=self.get_clock())

        # Desired setpoints.
        self.center_pixel = 320.0   # For a 640-pixel wide image.
        self.desired_distance = 1.0   # Desired distance (meters).

        # Current sensor measurements.
        self.current_pixel = None   # From 'detected_pixel'
        self.current_distance = None  # From 'object_range'

        # Control parameters.
        self.dead_zone = 10         # Ignore small pixel errors.
        self.max_angular_speed = 0.5  # Maximum angular velocity.
        self.max_linear_speed = 0.5   # Maximum linear velocity.

        # For timeout handling.
        self.last_pixel_msg_time = self.get_clock().now()
        self.last_range_msg_time = self.get_clock().now()
        self.timeout_duration = 1.0  # seconds

        # Timer for the control loop (10 Hz).
        self.timer = self.create_timer(0.1, self.control_loop)

    def pixel_callback(self, msg: Point):
        # Assume msg.x is the detected pixel value.
        self.current_pixel = msg.x
        self.last_pixel_msg_time = self.get_clock().now()

    def range_callback(self, msg: Point):
        # Assume msg.x is the measured distance (meters).
        self.current_distance = msg.x
        self.last_range_msg_time = self.get_clock().now()

    def control_loop(self):
        current_time = self.get_clock().now()
        # Check for timeouts.
        pixel_timeout = (current_time - self.last_pixel_msg_time).nanoseconds / 1e9 > self.timeout_duration
        range_timeout = (current_time - self.last_range_msg_time).nanoseconds / 1e9 > self.timeout_duration
        
        twist = Twist()

        # Angular control: use PID if pixel data is available and recent.
        if self.current_pixel is not None and not pixel_timeout:
            # Pixel error: positive if the object is to the left.
            pixel_error = self.center_pixel - self.current_pixel
            if abs(pixel_error) > self.dead_zone:
                angular_output = self.angular_pid.compute(pixel_error)
                # Saturate the output.
                angular_output = max(min(angular_output, self.max_angular_speed), -self.max_angular_speed)
                twist.angular.z = angular_output
            else:
                twist.angular.z = 0.0
        else:
            twist.angular.z = 0.0

        # Linear control: use PID if range data is available and recent.
        if self.current_distance is not None and not range_timeout:
            # Distance error: positive if object is too far.
            distance_error = self.desired_distance - self.current_distance
            linear_output = self.linear_pid.compute(distance_error)
            # Saturate the output.
            linear_output = max(min(linear_output, self.max_linear_speed), -self.max_linear_speed)
            twist.linear.x = linear_output
        else:
            twist.linear.x = 0.0

        self._vel_publish.publish(twist)

def main():
    rclpy.init()
    velocity_controller = VelocityController()

    while rclpy.ok():
        rclpy.spin_once(velocity_controller)
    
    velocity_controller.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()
