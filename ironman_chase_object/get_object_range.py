# Marilyn Braojos 
# Mariam Misabishvili

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

import cv2
from cv_bridge import CvBridge
import numpy as np
import math


class GetObjectRangeNode(Node):

    def __init__(self):        
        super().__init__('get_object_range_node')

        self.declare_parameter('camera_fov_deg', 62.2)  # Horizontal field of view in degrees
        self.declare_parameter('image_width', 320)      # Image width in pixels

        # Fetch parameter values
        self.camera_fov_deg = self.get_parameter('camera_fov_deg').value
        self.image_width = self.get_parameter('image_width').value

        # Calculate how many degrees each pixel represents (very rough approximation)
        self.angle_per_pixel = self.camera_fov_deg / float(self.image_width)

        self.object_x = None
        self.center_img = None

        lidar_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self._pix_subscriber = self.create_subscription(
            Point,
            'detected_pixel',
            self._pixel_callback, 
            10)
        self._pix_subscriber 

        self._lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self._distance_callback, 
            lidar_qos_profile)
        self._lidar_subscriber 
        
        self.object_distance_pub = self.create_publisher(Point, 'obj_dis', 10)

    def _pixel_callback(self, msg: Point):
        self.object_x = msg.x # object center in pixels
        self.center_img = msg.y # img center in pixels
        #self.get_logger().info(f"Received detected pixel: object_x = {self.object_x}, center_img = {self.center_img}")

    def _distance_callback(self, scan_msg: LaserScan):    
        if self.object_x is None: 
            return 

        pix_error = self.object_x - self.center_img
        angle_deg = pix_error * self.angle_per_pixel
        angle_rad = math.radians(angle_deg)

        if pix_error < 0:
            angle_rad = math.radians(angle_deg) + (2 * math.pi)

        # Find the indices within +/- 0.2 rad
        min_angle = angle_rad - 0.1
        max_angle = angle_rad + 0.1

        indices = [i for i in range(len(scan_msg.ranges)) 
                   if scan_msg.angle_min + i * scan_msg.angle_increment >= min_angle 
                   and scan_msg.angle_min + i * scan_msg.angle_increment <= max_angle]
        print(indices)
        # Get the average distance
        valid_ranges = [scan_msg.ranges[i] for i in indices if scan_msg.ranges[i] > 0.0]
        
        if valid_ranges:
            distance = sum(valid_ranges) / len(valid_ranges)
            point = Point()
            point.y = distance
            self.object_distance_pub.publish(point)      

        # debug :((
        self.get_logger().info(f"pix_error: {pix_error}")
        self.get_logger().info(f"angle_deg: {angle_deg}")
        self.get_logger().info(f"angle_rad: {angle_rad}")
        self.get_logger().info(f"Indices in window: {indices}")
        raw_distances = [scan_msg.ranges[i] for i in indices if scan_msg.ranges[i] > 0.0]
        self.get_logger().info(f"Raw distances: {raw_distances}")

 
        # else: 
        #     index = int(angle_deg + 360)

        # if 0 <= index < len(scan_msg.ranges):
        #     distance = scan_msg.ranges[index]
        # else:
        #     distance = float('inf') 
        #     self.get_logger().warn(f"target angle {angle_rad} is not between {scan_msg.angle_min} and {scan_msg.angle_max}")

        # out_msg = Point()
        # out_msg.x = angle_rad
        # out_msg.y = distance
        # out_msg.z = 0.0

        # self.object_distance_pub.publish(out_msg)

        # if scan_msg.angle_min <= target_angle <= scan_msg.angle_max: 
        #     index = int((target_angle - scan_msg.angle_min)/scan_msg.angle_increment)
        # else: 
        #     
        #     return 
        
        # # old
        # # angle_deg_normalized = angle_deg % 360.0
        # # index = int(angle_deg_normalized)

        # # index = angle_deg_normalized

        # # Check that index is within LIDAR range array
        # 
        #    
        # # Optional: Try checking neighboring indices if you're worried about off-by-one errors
        # if distance == float('inf') and 0 <= index - 5 < len(scan_msg.ranges):
        #     distance = scan_msg.ranges[index - 5]

        # if distance == float('inf') and 0 <= index + 5 < len(scan_msg.ranges):
        #     distance = scan_msg.ranges[index + 5]

        # #    Use geometry_msgs/Point where:
        # 
        # # out_msg.x = math.radians(angle_deg_normalized)  # angle in radians
        # out_msg.x = target_angle
        # out_msg.y = distance
        # out_msg.z = 0.0

        # self.object_distance_pub.publish(out_msg)

def main():
    rclpy.init()
    lidar_subscriber = GetObjectRangeNode()

    while rclpy.ok():
        rclpy.spin_once(lidar_subscriber)
    
    lidar_subscriber.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()