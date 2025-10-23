import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class Minitask1(Node):

    def __init__(self):
        super().__init__('minitask1')
        # Create the publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create the subscriber for odometry data
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # State machine variables
        self.state = 'FORWARD'  # States: FORWARD, TURN, STOP
        self.side_count = 0  # Track which side of square we're on
        
        # Movement parameters
        self.forward_speed = 0.2  # m/s
        self.turn_speed = 0.3  # rad/s 
        self.side_length = 1.0  # meters
        
        # Odometry tracking variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0  # orientation in radians
        
        # Target tracking
        self.start_x = None
        self.start_y = None
        self.target_yaw = 0.0
        
        # Tolerance thresholds avoids overshooting
        self.distance_tolerance = 0.05  # 5cm
        self.angle_tolerance = 0.05  # ~3 degrees

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_distance_traveled(self):
        """Calculate distance traveled from start position"""
        if self.start_x is None or self.start_y is None:
            return 0.0
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx * dx + dy * dy)

    def get_angle_difference(self):
        """Calculate difference between current yaw and target yaw"""
        diff = self.target_yaw - self.current_yaw
        return self.normalize_angle(diff)

    def timer_callback(self):
        """Called every timer_period to control robot movement"""
        # Wait until we have odometry data
        if self.start_x is None:
            return
        
        # Create new Twist message
        msg = Twist()
        msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        
        # State machine for square movement
        if self.state == 'FORWARD':
            distance_traveled = self.get_distance_traveled()
            
            if distance_traveled < self.side_length - self.distance_tolerance:
                # Keep moving forward
                msg.linear.x = self.forward_speed
                msg.angular.z = 0.0
            else:
                # Reached target distance, stop and prepare to turn
                self.get_logger().info(f'Completed side {self.side_count + 1}/4 - Distance: {distance_traveled:.3f}m')
                self.state = 'TURN'
                # Set target yaw for next turn (90 degrees counter-clockwise)
                self.target_yaw = self.normalize_angle(self.current_yaw + math.pi / 2)
            
        elif self.state == 'TURN':
            angle_diff = self.get_angle_difference()
            
            if abs(angle_diff) > self.angle_tolerance:
                # Keep turning
                msg.linear.x = 0.0
                # Turn in direction of shortest path
                msg.angular.z = self.turn_speed if angle_diff > 0 else -self.turn_speed
            else:
                # Completed turn
                self.side_count += 1
                self.get_logger().info(f'Completed turn {self.side_count}/4 - Yaw: {math.degrees(self.current_yaw):.1f}°')
                
                # Check if we've completed the square
                if self.side_count >= 4:
                    self.get_logger().info('Square completed! Stopping.')
                    self.state = 'STOP'
                else:
                    # Reset for next side
                    self.start_x = self.current_x
                    self.start_y = self.current_y
                    self.state = 'FORWARD'
        
        elif self.state == 'STOP':
            # Stop the robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.timer.cancel()
            return
        
        # Publish the movement command
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        """Callback for odometry data - update current position and orientation"""
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract orientation (quaternion) and convert to Euler angles
        orientation = msg.pose.pose.orientation
        roll, pitch, self.current_yaw = self.quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        
        # Initialize start position on first callback
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.target_yaw = self.current_yaw
            self.get_logger().info(f'Starting position: x={self.current_x:.3f}, y={self.current_y:.3f}')

def main(args=None):
    rclpy.init(args=args)
    mt = Minitask1()
    rclpy.spin(mt)
    mt.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()