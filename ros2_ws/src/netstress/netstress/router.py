import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CountingListener:
    def __init__(self, node, topic_name):
        self.node = node
        self.count = 0
        self.subscription = node.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.count += 1
        self.node.get_logger().info(f"Received: '{msg.data}', total count: {self.count}")

class CountingPublisher:
    def __init__(self, node, topic_name):
        self.node = node
        self.count = 0
        self.publisher = node.create_publisher(String, topic_name, 10)
        self.timer = node.create_timer(0.1, self.publish_msg)

    def publish_msg(self):
        msg = String()
        msg.data = f"Stress message {self.count}"
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published: '{msg.data}'")
        self.count += 1

class Router(Node):
    def __init__(self):
        super().__init__('router')
        self.output = CountingPublisher(self, 'net_test')
        self.input = CountingListener(self, 'net_test')

def main(args=None):
    rclpy.init(args=args)
    node = Router()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

