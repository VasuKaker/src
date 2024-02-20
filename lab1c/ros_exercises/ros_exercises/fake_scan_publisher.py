import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import random


class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.declare_parameter('publish_topic', 'fake_scan')
        self.declare_parameter('publish_rate', 0.05)
        self.declare_parameter('angle_min', -2.0/3.0*np.pi)
        self.declare_parameter('angle_max', 2.0/3.0*np.pi)
        self.declare_parameter('angle_increment', 1.0/300.0*np.pi)
        self.declare_parameter('range_min', 1.0)
        self.declare_parameter('range_max', 10.0)
        self.publisher_ = self.create_publisher(LaserScan, 'fake_scan', 10)
        timer_period = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.second_publisher = self.create_publisher(Float32, 'range_test', 10)


    def timer_callback(self):
        msg = LaserScan()
        msg_two = Float32()

        # Populate the LaserScan message
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "base_link"
        
        msg.angle_min = self.get_parameter('angle_min').value
        msg.angle_max = self.get_parameter('angle_max').value
        msg.angle_increment = self.get_parameter('angle_increment').value
        msg.range_min = self.get_parameter('range_min').value
        msg.range_max = self.get_parameter('range_max').value
        
        # Calculate number of measurements
        num_measurements = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [float(random.uniform(1.0, 10.0)) for _ in range(num_measurements)]

        msg_two.data = float(len(msg.ranges))
        self.publisher_.publish(msg)
        self.second_publisher.publish(msg_two)
        # self.get_logger().info('Publishing fake scan data: ')


def main(args=None):
    rclpy.init(args=args)
    fake_scan_publisher = FakeScanPublisher()
    rclpy.spin(fake_scan_publisher)
    
    fake_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
