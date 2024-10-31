# robot_controller.py
import rclpy
from rclpy.node import Node
from typing import List
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AutonomousRobotController(Node):
    def __init__(self) -> None:
        super().__init__('autonomous_robot_controller')
        
        # Define Publisher for movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define Subscriber for LIDAR data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Parameters for movement
        self.forward_speed: float = 0.2  # m/s
        self.turn_speed: float = 1.0     # rad/s
        self.safe_distance: float = 0.5  # meters
        
        self.get_logger().info('Autonomous Robot Controller Node started.')

    def lidar_callback(self, msg: LaserScan) -> None:
        # Check if an obstacle is within the safe distance
        obstacles: List[float] = [distance for distance in msg.ranges if distance < self.safe_distance]
        
        # If obstacles are detected, rotate. Otherwise, move forward.
        if obstacles:
            self.rotate()
        else:
            self.move_forward()

    def move_forward(self) -> None:
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Moving forward.')

    def rotate(self) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.turn_speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Rotating to avoid obstacle.')

def main(args=None) -> None:
    rclpy.init(args=args)
    controller = AutonomousRobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
