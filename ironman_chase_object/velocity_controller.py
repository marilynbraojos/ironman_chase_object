import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import math

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        
        # Subscriber to object_range topic, which publishes a Point:
        #   - msg.x: angular offset (radians)
        #   - msg.y: distance (meters)
        self._range_subscriber = self.create_subscription(
            Point,
            'object_range',
            self.range_callback,
            10)
        
        # Publisher for velocity commands
        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ----- Angular PID Parameters (for rotation) -----
        self.Kp = 0.005       # Proportional gain (angular)
        self.Ki = 0.0001      # Integral gain (angular)
        self.Kd = 0.001       # Derivative gain (angular)
        
        self.prev_ang_error = 0.0  # previous angular error (radians)
        self.sum_ang_error = 0.0   # accumulated angular error
        # Dead zone (in radians) to ignore small angular errors:
        self.dead_zone = math.radians(1.0)  # ~1 degree
        
        self.max_angular_speed = 1.0  # Maximum angular speed (rad/s)
        
        # ----- Linear PID Parameters (for forward/backward motion) -----
        self.kp_linear = 0.1    # Proportional gain (linear)
        self.ki_linear = 0.005  # Integral gain (linear)
        self.kd_linear = 0.01   # Derivative gain (linear)
        
        self.prev_lin_error = 0.0   # previous linear error (meters)
        self.sum_lin_error = 0.0    # accumulated linear error
        
        self.desired_distance = 0.75  # Desired distance from the object (meters)
        self.max_linear_speed = 0.5   # Maximum linear speed (m/s)
        
        # ----- Timing Variables for PID (dt calculation) -----
        self.last_time = self.get_clock().now()
        
        # Latest object_range message storage
        self.latest_range_msg = None
        
        # Control loop timer (~20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # ----- Timeout Safety -----
        self.last_msg_time = self.get_clock().now()  # time of last received message
        self.timeout_duration = 1.0  # seconds before stopping if no messages arrive
        self.timeout_timer = self.create_timer(0.5, self.check_timeout)
    
    def range_callback(self, msg: Point):
        """
        Callback for the 'object_range' topic.
        Assumptions:
          - msg.x: angular offset (in radians) from desired center (target = 0)
          - msg.y: measured distance (in meters) from the object
        """
        self.latest_range_msg = msg
        self.last_msg_time = self.get_clock().now()
    
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9  # dt in seconds
        self.last_time = now
        
        twist = Twist()
        
        if self.latest_range_msg is not None:
            # ----- Angular PID Controller -----
            # Error: measured angle (msg.x) should be zero.
            ang_error = self.latest_range_msg.x
            if abs(ang_error) > self.dead_zone:
                # Accumulate error for integral term.
                self.sum_ang_error += ang_error * dt
                # Derivative (rate of change of error).
                d_ang_error = (ang_error - self.prev_ang_error) / dt if dt > 0 else 0.0
                # PID output calculation:
                angular_output = self.Kp * ang_error + self.Ki * self.sum_ang_error + self.Kd * d_ang_error
                # Clamp to max angular speed.
                angular_output = max(-self.max_angular_speed, min(angular_output, self.max_angular_speed))
                # Note: The negative sign here follows your original convention.
                twist.angular.z = -angular_output
                self.prev_ang_error = ang_error
            else:
                twist.angular.z = 0.0
            
            # ----- Linear PID Controller -----
            # Error: measured distance (msg.y) compared to desired distance.
            measured_distance = self.latest_range_msg.y
            lin_error = measured_distance - self.desired_distance
            self.sum_lin_error += lin_error * dt
            d_lin_error = (lin_error - self.prev_lin_error) / dt if dt > 0 else 0.0
            linear_output = (self.kp_linear * lin_error +
                             self.ki_linear * self.sum_lin_error +
                             self.kd_linear * d_lin_error)
            # Clamp to maximum linear speed.
            linear_output = max(-self.max_linear_speed, min(linear_output, self.max_linear_speed))
            # Depending on your robot's configuration, you might need to negate the output.
            twist.linear.x = -linear_output
            self.prev_lin_error = lin_error
        
        self._vel_publish.publish(twist)
    
    def check_timeout(self):
        """If no new messages are received for 'timeout_duration' seconds, stop the robot."""
        time_since_last = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if time_since_last > self.timeout_duration:
            twist = Twist()
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self._vel_publish.publish(twist)

def main():
    rclpy.init()
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()