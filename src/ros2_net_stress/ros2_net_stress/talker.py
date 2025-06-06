import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StressTalker(Node):
    def __init__(self):
        super().__init__('stress_talker')
        self.publisher = self.create_publisher(String, 'net_test', 10)
        self.count = 0
        self.timer = self.create_timer(0.1, self.publish_msg)

    def publish_msg(self):
        msg = String()
        msg.data = f"Stress message {self.count}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: '{msg.data}'")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = StressTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down from Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

