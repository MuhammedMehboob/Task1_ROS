import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class Minitask1(Node):

    def __init__(self):
        super().__init__('minitask1')
        # Create the publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds - check state more frequently
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create the subscriber for odometry data (just for logging)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # State machine variables
        self.state = 'FORWARD'  # States: FORWARD, TURN, STOP
        self.side_count = 0  # Track which side of square we're on
        self.state_timer = 0.0  # Track time in current state
        
        # Movement parameters - tune these for your robot
        self.forward_speed = 0.2  # m/s (linear velocity)
        self.turn_speed = 0.5  # rad/s (angular velocity)
        self.side_length = 1.0  # meters (length of each side of square)
        self.turn_angle = 1.5708  # 90 degrees in radians (pi/2)
        
        # Calculate durations based on speed and distance/angle
        # Time = Distance / Speed
        self.forward_duration = self.side_length / self.forward_speed
        # Time = Angle / Angular_Speed
        self.turn_duration = self.turn_angle / self.turn_speed
        
        self.get_logger().info(f'Starting open-loop square movement')
        self.get_logger().info(f'Forward duration: {self.forward_duration:.2f}s, Turn duration: {self.turn_duration:.2f}s')

    def timer_callback(self):
        """Called every timer_period to control robot movement using time-based control"""
        # Create new Twist message
        msg = Twist()
        msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        
        # State machine for square movement (open-loop - time-based only)
        if self.state == 'FORWARD':
            # Drive forward at constant speed
            msg.linear.x = self.forward_speed
            msg.angular.z = 0.0
            
            # Check if we've driven for the calculated time
            if self.state_timer >= self.forward_duration:
                # Switch to turning
                self.state = 'TURN'
                self.state_timer = 0.0
                self.get_logger().info(f'Completed side {self.side_count + 1}/4, starting turn')
            
        elif self.state == 'TURN':
            # Turn in place at constant angular speed
            msg.linear.x = 0.0
            msg.angular.z = self.turn_speed  # positive = counter-clockwise (left turn)
            
            # Check if we've turned for the calculated time
            if self.state_timer >= self.turn_duration:
                self.side_count += 1
                
                # Check if we've completed all 4 sides of the square
                if self.side_count >= 4:
                    self.get_logger().info('Square completed! Stopping robot.')
                    self.state = 'STOP'
                else:
                    # Move to next side
                    self.state = 'FORWARD'
                    self.state_timer = 0.0
                    self.get_logger().info(f'Completed turn {self.side_count}/4, moving forward')
        
        elif self.state == 'STOP':
            # Stop the robot completely
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            # Cancel the timer to stop calling this function
            self.timer.cancel()
            return
        
        # Increment state timer by the timer period
        self.state_timer += 0.1  # Must match timer_period
        
        # Publish the movement command
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        """Callback for odometry data - only used for logging position"""
        location = msg.pose.pose.position
        # Only log occasionally to avoid spam
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
            
        if self._log_counter % 10 == 0:  # Log every 10th message
            self.get_logger().info(f'Robot position: x={location.x:.2f}m, y={location.y:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    mt = Minitask1()
    rclpy.spin(mt)
    mt.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()