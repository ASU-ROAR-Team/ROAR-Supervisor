import rclpy
from rclpy.node import Node
from supervisor.heartbeat_publisher import *
from rclpy.executors import MultiThreadedExecutor

class my_first_node(Node):
    def __init__(self):
        super().__init__("my_first_node")
        self.get_logger().info("Hello from Python ROS2 Node!")
        self.heartbeat = HeartbeatPublisher(message="my_first_node", interval=2)

def main(args=None):
    rclpy.init(args=args)
    node = my_first_node()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.heartbeat)
    
    try:
        executor.spin()
    finally:
        node.heartbeat.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
