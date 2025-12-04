import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import time

class HeartbeatPublisher(Node):
    def __init__(self, topic="heartbeat", message="alive", interval=1.0):
        """
        topic:     topic name to publish heartbeat on
        message:   heartbeat message text
        interval:  heartbeat frequency in seconds
        """
        super().__init__(f"heartbeat_node_{message.replace(' ', '_')}")
        self.publisher = self.create_publisher(String, topic, 10)
        self.msg_text = message
        self.timer = self.create_timer(interval, self.publish_heartbeat)
        self.get_logger().info(
            f"Heartbeat started: topic={topic}, message='{message}', interval={interval}s"
        )

    def publish_heartbeat(self):
        msg = String()
        msg.data = self.msg_text
        self.publisher.publish(msg)
        self.get_logger().debug(f"Heartbeat: {msg.data}")