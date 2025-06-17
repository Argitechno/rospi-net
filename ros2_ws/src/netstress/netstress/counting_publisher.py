from std_msgs.msg import String

class CountingPublisher:
    """A ROS2 publisher that publishes messages with an incrementing count on a specified topic."""
    
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