#my_second_node.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

class my_second_node(Node):
    def __init__(self):
        super().__init__("my_second_node")
        self.get_logger().info("Hello from Python ROS2 Node!")

def main(args=None):
    rclpy.init(args=args)
    node = my_second_node()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
