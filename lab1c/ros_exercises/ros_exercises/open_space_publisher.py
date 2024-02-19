import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from std_msgs.msg import Float32
import numpy as np

class OpenSpacePublisher(Node):
    def __init__(self):
        super().__init__('open_space_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.listener_callback,
            10)
        self.publisher_distance = self.create_publisher(Float32, 'open_space/distance', 10)
        self.publisher_angle = self.create_publisher(Float32, 'open_space/angle', 10)
        self.timer = self.create_timer(1/20, self.publish_data)
        self.longest_range = None
        self.angle_of_longest_range = None

    def listener_callback(self, msg):
        print("we made it here")
        ranges = np.array(msg.ranges)
        print("we made it here 2")
        max_range_index = np.argmax(ranges)
        self.longest_range = ranges[max_range_index]
        angle_increment = msg.angle_increment
        self.angle_of_longest_range = msg.angle_min + max_range_index * angle_increment

    def publish_data(self):
        print("we are publishing now")
        if self.longest_range is not None and self.angle_of_longest_range is not None:
            distance_msg = Float32()
            print("self.longest_range is: ", self.longest_range)
            distance_msg.data = self.longest_range
            print("just before the .publish line")
            self.publisher_distance.publish(distance_msg)
            print("distance message published")

            angle_msg = Float32()
            angle_msg.data = self.angle_of_longest_range
            self.publisher_angle.publish(angle_msg)
            print("publisher angle published ")
            # Log the published data
            self.get_logger().info(f'Longest Range: {self.longest_range}, Angle: {self.angle_of_longest_range}')

def main(args=None):
    rclpy.init(args=args)
    open_space_publisher = OpenSpacePublisher()
    rclpy.spin(open_space_publisher)
    open_space_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
