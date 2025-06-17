from std_msgs.msg import String

class CountingSubscriber:
    """A ROS2 subscriber that counts the number of messages received on a specified topic."""
    
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