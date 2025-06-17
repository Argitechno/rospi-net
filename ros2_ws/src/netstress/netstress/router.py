import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import String

class CountingListener:
    def __init__(self, node, topic_name):
        self.node = node
        self.topic_name = topic_name
        self.count = 0
        self.subscription = node.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.count += 1
        self.node.get_logger().info(f"[{self.topic_name}] Received: '{msg.data}', total count: {self.count}")

class CountingPublisher:
    def __init__(self, node, topic_name):
        self.node = node
        self.topic_name = topic_name
        self.count = 0
        self.publisher = node.create_publisher(String, topic_name, 10)
        self.timer = node.create_timer(0.1, self.publish_msg)

    def publish_msg(self):
        msg = String()
        msg.data = f"Stress message {self.count}"
        self.publisher.publish(msg)
        self.node.get_logger().info(f"[{self.topic_name}] Published: '{msg.data}'")
        self.count += 1

class Router(Node):
    def __init__(self):
        super().__init__('router')

        # Declare parameters with default empty lists
        self.declare_parameter(
            'talk_topics',
            value=[''],
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            'listen_topics',
            value=[''],
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )

        # Get parameter values as string arrays
        self.talk_topics = [t for t in self.get_parameter('talk_topics').get_parameter_value().string_array_value if t]
        self.listen_topics = [t for t in self.get_parameter('listen_topics').get_parameter_value().string_array_value if t]

        self.get_logger().info(f"Talk topics: {self.talk_topics}")
        self.get_logger().info(f"Listen topics: {self.listen_topics}")

        if not self.talk_topics:
            self.get_logger().warn("No valid talk_topics specified.")
        if not self.listen_topics:
            self.get_logger().warn("No valid listen_topics specified.")

        # Create CountingPublisher instances for each talk topic
        self.counting_publishers = {}
        for topic in self.talk_topics:
            self.counting_publishers[topic] = CountingPublisher(self, topic)

        # Create CountingListener instances for each listen topic
        self.counting_listeners = {}
        for topic in self.listen_topics:
            self.counting_listeners[topic] = CountingListener(self, topic)

def main(args=None):
    rclpy.init(args=args)
    node = Router()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

