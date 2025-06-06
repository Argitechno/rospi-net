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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down from Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
