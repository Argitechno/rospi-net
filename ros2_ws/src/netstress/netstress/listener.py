import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StressListener(Node):
    def __init__(self):
        super().__init__('stress_listener')
        self.subscription = self.create_subscription(
            String,
            'net_test',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    node = StressListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
