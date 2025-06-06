import rclpy
from rclpy.parameter import Parameter
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

        # Declare parameters with default values (empty list)
        self.declare_parameter('talk_topics', [''])
        self.declare_parameter('listen_topics', [''])

        # Read parameter values
        self.talk_topics = self.get_parameter('talk_topics').get_parameter_value().string_array_value
        self.listen_topics = self.get_parameter('listen_topics').get_parameter_value().string_array_value

        self.get_logger().info(f"Talk topics: {self.talk_topics}")
        self.get_logger().info(f"Listen topics: {self.listen_topics}")
        
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

