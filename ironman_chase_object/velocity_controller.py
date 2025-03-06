import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Point

class VelocityController(Node):
    def __init__(self):        
        super().__init__('velocity_publisher')

        # Subscribe to the detected pixel topic.
        self._pixel_subscriber = self.create_subscription(
            Point,
            'detected_pixel',
            self.pixel_callback, 
            10)
        self._pixel_subscriber 

        self.dis_subscriber = self.create_subscription(
            Point,
            'obj_dis',
            self.lidar_callback, 
            10)
        self.dis_subscriber 
        
        # Publisher for velocity commands.
        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID gains for angular control.
        self.Kp = 0.005    # Proportional gain 
        self.Ki = 0.001    # Integral gain
        self.Kd = 0.001    # Derivative gain

        self.max_angular_speed = 1.0 
        self.dead_zone = 10  # Pixels within which we don't rotate

        # Initialize error tracking for PID.
        self.last_error = 0.0
        self.integral = 0.0
        self.last_update_time = self.get_clock().now()

        # Timeout mechanism.
        self.last_msg_time = self.get_clock().now()  # Track last received message time
        self.timeout_duration = 1.0  # Time (in seconds) before stopping motion
        self.timer = self.create_timer(0.5, self.check_timeout)

    def pixel_callback(self, msg: Point):
        current_time = self.get_clock().now()
        self.last_msg_time = current_time  # Update last received message time

        # Calculate pixel error: msg.x = detected pixel, msg.y = center pixel.
        error = msg.x - msg.y

        twist = Twist()
        twist.linear.x = 0.0

        dt = (current_time - self.last_update_time).nanoseconds / 1e9  # Time difference in seconds
        
        # Only compute PID if the error exceeds the dead zone.
        if abs(error) > self.dead_zone and dt > 0.0:
            # Accumulate the integral term.
            self.integral += error * dt
            # Compute the derivative term.
            derivative = (error - self.last_error) / dt

            # Compute the full PID control output.
            output = - (self.Kp * error + self.Ki * self.integral + self.Kd * derivative)
            # Clamp the angular velocity.
            output = max(min(output, self.max_angular_speed), -self.max_angular_speed)
            twist.angular.z = output

            # Update error for next iteration.
            self.last_error = error
            self.last_update_time = current_time
        else:
            # If within dead zone, no rotation.
            twist.angular.z = 0.0
        
        self._vel_publish.publish(twist)

    def lidar_callback(self, out_msg: Point): 
        distance = out_msg.y

        print(distance)
        # self.get_logger().info(f"Angle min: {out_msg.angle_min} rad")
        # self.get_logger().info(f"Angle max: {out_msg.angle_max} rad")

        
    def check_timeout(self): 
        """Stop rotation if no new message is received for `timeout_duration` seconds."""
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