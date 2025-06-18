from rclpy import init, spin, shutdown
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from netstress.counting_publisher import CountingPublisher
from netstress.counting_subscriber import CountingSubscriber

class Router(Node):
    def __init__(self):
        super().__init__('router')
       
        # Declare parameters with default empty lists
        self.declare_parameter(
            'publish_topics',
            value=[''],
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        
        self.declare_parameter(
            'subscribe_topics',
            value=[''],
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        
        default_publish_rate = 3.0
        
        self.declare_parameter(
            'default_publish_rate',
            value=default_publish_rate
        )
        
        self.declare_parameter(
            'publish_rates',
            value=[default_publish_rate],
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        

        # Get parameter values as string arrays
        self.publish_topics = [t for t in self.get_parameter_or('publish_topics', Parameter('publish_topics', Parameter.Type.STRING_ARRAY, [''])).get_parameter_value().string_array_value if t]
        self.subscribe_topics = [t for t in self.get_parameter_or('subscribe_topics', Parameter('subscribe_topics', Parameter.Type.STRING_ARRAY, [''])).get_parameter_value().string_array_value if t]
        
        self.default_publish_rate = self.get_parameter('default_publish_rate').get_parameter_value().double_value
        self.publish_rates = [t for t in self.get_parameter_or('publish_rates', Parameter('publish_rates', Parameter.Type.DOUBLE_ARRAY, [self.default_publish_rate])).get_parameter_value().double_array_value if t]
        
        self.get_logger().info(f"Publish topics: {self.publish_topics}")
        self.get_logger().info(f"Subscribe topics: {self.subscribe_topics}")

        if not self.publish_topics:
            self.get_logger().warn("No valid publish_topics specified.")
        if not self.subscribe_topics:
            self.get_logger().warn("No valid subscribe_topics specified.")

         # Match rates to topics
        self.publish_params = []
        for idx, topic in enumerate(self.publish_topics):
            if idx < len(self.publish_rates):
                rate = self.publish_rates[idx]
            else:
                rate = self.default_publish_rate
            self.publish_params.append((topic, rate))

        # Create CountingPublisher instances for each publish topic
        self.counting_publishers = {}
        for topic, rate in self.publish_params:
            self.counting_publishers[topic] = CountingPublisher(self, topic, rate)

        # Create CountingSubscriber instances for each subscribe topic
        self.counting_subscribers = {}
        for topic in self.subscribe_topics:
            self.counting_subscribers[topic] = CountingSubscriber(self, topic)

def main(args=None):
    init(args=args)
    node = Router()
    spin(node)
    node.destroy_node()
    shutdown()

if __name__ == '__main__':
    main()

