import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'my_random_float',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'random_float_log', 10)


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        msg_two = Float32()
        msg_two.data = np.log(msg.data)
        self.publisher_.publish(msg_two)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

