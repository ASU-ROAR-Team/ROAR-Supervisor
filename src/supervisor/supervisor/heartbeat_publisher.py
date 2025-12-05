import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json


class HeartbeatPublisher(Node):
    def __init__(self, process_name, heartbeat_topic="heartbeat", resource_topic="resource", message="alive", interval=1.0, pid=None):
        """
        topic:     topic name to publish heartbeat and/or resource usage
        message:   heartbeat message text
        interval:  publish frequency in seconds
        pid:       process ID to monitor CPU/mem usage
        """
        super().__init__(f"heartbeat_node_{message.replace(' ', '_')}")
        self.heartbeat_publisher = self.create_publisher(String, heartbeat_topic, 10)
        self.resource_publisher = self.create_publisher(String, resource_topic, 10)
        self.msg_text = message
        self.pid = pid
        self.process_name = process_name
        self.process = None
        self.timer = self.create_timer(interval, self.timer_callback)

        self.get_logger().info(
            f"Monitoring started for {process_name}, PID={pid}, interval={interval}s"
        )

    def timer_callback(self):
        """Called periodically to publish both heartbeat and resource usage."""
        self.publish_heartbeat()
        self.publish_resource_usage()

    def publish_heartbeat(self):
        """Publish a simple heartbeat message."""
        msg = String()
        msg.data = self.msg_text
        self.heartbeat_publisher.publish(msg)
        self.get_logger().debug(f"Heartbeat: {msg.data}")

    def publish_resource_usage(self):
        """Publish CPU, memory, and process state as JSON."""
        msg = String()
        
        if self.process is None:
            try:
                self.process = psutil.Process(self.pid)
            except psutil.NoSuchProcess:
                data = {
                    "process_name": self.process_name,
                    "pid": self.pid,
                    "status": "not_found",
                    "cpu_percent": 0.0,
                    "memory_mb": 0.0,
                    "alive": False
                }
                msg.data = json.dumps(data)
                self.resource_publisher.publish(msg)
                return
        
        try:
            # Get process status
            status = self.process.status()
            
            # Get CPU percentage
            cpu_percent = self.process.cpu_percent(interval=1)
            
            # Get memory usage in MB
            memory_mb = self.process.memory_info().rss / (1024 * 1024)
            
            data = {
                "process_name": self.process_name,
                "pid": self.pid,
                "status": status,
                "cpu_percent": round(cpu_percent, 2),
                "memory_mb": round(memory_mb, 2),
                "alive": True
            }
            msg.data = json.dumps(data)
            
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            # Process died or no access
            data = {
                "process_name": self.process_name,
                "pid": self.pid,
                "status": "terminated",
                "cpu_percent": 0.0,
                "memory_mb": 0.0,
                "alive": False
            }
            msg.data = json.dumps(data)
            self.process = None
        
        self.resource_publisher.publish(msg)
        self.get_logger().debug(f"Resource: {msg.data}")

    def shutdown(self):
        if self.timer:
            self.timer.cancel()
        self.destroy_node()