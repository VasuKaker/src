import rclpy
from rclpy.node import Node
import numpy as np

# from std_msgs.msg import LaserScan
from sensor_msgs.msg import LaserScan
import random


class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'fake_scan', 10)
        timer_period = 0.05  # seconds, which corresponds to 20Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()

        # Populate the LaserScan message
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "base_link"
        msg.angle_min = -2.0/3.0*np.pi
        msg.angle_max = 2.0/3.0*np.pi
        msg.angle_increment = 1.0/300.0*np.pi
        msg.range_min = 1.0
        msg.range_max = 10.0
        
        # Calculate number of measurements
        num_measurements = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [float(random.uniform(1.0, 10.0)) for _ in range(num_measurements)]

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing fake scan data: ')


def main(args=None):
    rclpy.init(args=args)
    fake_scan_publisher = FakeScanPublisher()
    rclpy.spin(fake_scan_publisher)
    
    fake_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
