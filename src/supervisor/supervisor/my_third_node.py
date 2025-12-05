import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import time
import os

class StressNode(Node):
    def __init__(self):
        super().__init__('stress_node')

        self.publisher = self.create_publisher(String, 'stress_status', 10)

        self.memory_target_mb = 500  # Allocate 500 MB
        self.memory_data = []

        self.get_logger().info(
            f"StressNode started. PID={os.getpid()}"
        )

        # Allocate memory immediately
        chunk = np.random.rand(1000, 1000)  # ~8 MB
        total_alloc = 0
        while total_alloc < self.memory_target_mb:
            self.memory_data.append(np.copy(chunk))
            total_alloc += chunk.nbytes / (1024 * 1024)
        self.get_logger().info(f"Allocated ~{total_alloc:.1f} MB memory")

        # Start timer to do CPU-intensive task
        self.create_timer(0.01, self.cpu_stress)
        self.create_timer(1.0, self.status_publish)

    def cpu_stress(self):
        """Do a CPU-heavy NumPy operation to keep CPU busy"""
        a = np.random.rand(500, 500)
        b = np.random.rand(500, 500)
        np.dot(a, b)  # heavy computation

    def status_publish(self):
        msg = String()
        msg.data = "StressNode running..."
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StressNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
