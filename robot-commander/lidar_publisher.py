# lidar_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random
from typing import List

class LidarPublisher(Node):
    def __init__(self) -> None:
        super().__init__('lidar_publisher')
        
        # Publisher for the simulated LIDAR data
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        # Timer to publish at a regular interval
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.publish_lidar_scan)
        
        # LIDAR parameters
        self.range_min: float = 0.1  # Minimum range in meters
        self.range_max: float = 10.0  # Maximum range in meters
        self.num_readings: int = 360  # Number of readings per scan
        self.angle_increment: float = 2.0 * math.pi / self.num_readings  # Angle increment per reading
        
        self.get_logger().info('LIDAR Publisher Node started.')

    def publish_lidar_scan(self) -> None:
        scan = LaserScan()
        
        # Fill LaserScan message fields
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "lidar_frame"
        
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = self.angle_increment
        scan.time_increment = self.timer_period / self.num_readings
        scan.scan_time = self.timer_period
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Generate simulated distance measurements with some noise
        scan.ranges = [self.generate_random_range() for _ in range(self.num_readings)]
        
        # Publish the scan
        self.lidar_publisher.publish(scan)
        self.get_logger().info('Published LIDAR scan data.')

    def generate_random_range(self) -> float:
        # Simulate a random distance with some noise, simulating obstacles at random distances
        return random.uniform(self.range_min, self.range_max)

def main(args=None) -> None:
    rclpy.init(args=args)
    lidar_publisher = LidarPublisher()
    rclpy.spin(lidar_publisher)
    lidar_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
