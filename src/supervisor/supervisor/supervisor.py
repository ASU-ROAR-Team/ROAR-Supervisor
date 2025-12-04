import rclpy
import threading
import yaml
import importlib
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from supervisor.heartbeat_publisher import HeartbeatPublisher
import os
from ament_index_python.packages import get_package_share_directory

class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")
        self.get_logger().info("Supervisor running")

# Helper function to import a class dynamically from a string path
def import_class(full_class_string):
    module_name, class_name = full_class_string.rsplit(".", 1)
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    return cls

#Input thread for selecting missions
def input_thread(executor, missions, node_classes, active_nodes):
    while True:
        cmd = input(f"Choose a mission {list(missions.keys())} or 'exit': ").strip()
        if cmd == "exit":
            print("Shutting down...")
            rclpy.shutdown()
            break
        elif cmd in missions:
            # Destroy previously active nodes and their heartbeats
            for node_data in active_nodes.values():
                node_instance = node_data['node']
                heartbeat = node_data.get('heartbeat')
                executor.remove_node(node_instance)
                if heartbeat:
                    executor.remove_node(heartbeat)
                    heartbeat.destroy_node()
                node_instance.destroy_node()
            active_nodes.clear()

            # Launch new nodes for this mission
            for node_name in missions[cmd]:
                cls = node_classes.get(node_name)
                if cls:
                    node_instance = cls()
                    executor.add_node(node_instance)
                    # Also add the heartbeat if it exists
                    heartbeat = getattr(node_instance, 'heartbeat', None)
                    if heartbeat:
                        executor.add_node(heartbeat)
                    active_nodes[node_name] = {'node': node_instance, 'heartbeat': heartbeat}
            print(f"Mission '{cmd}' launched: {missions[cmd]}")
        else:
            print("Unknown mission")

def main(args=None):
    rclpy.init(args=args)
    
    config_path = os.path.join(get_package_share_directory('supervisor'), 'config.yaml')

    # Load YAML
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    
    missions = config.get("missions", {})
    nodes = config.get("nodes", {})

    NODE_CLASSES = {name: import_class(path) for name, path in nodes.items()}

    supervisor_node = Supervisor()
    heartbeat_node = HeartbeatPublisher(message="Supervisor", interval=5)

    executor = MultiThreadedExecutor()
    executor.add_node(supervisor_node)
    executor.add_node(heartbeat_node)

    active_nodes = {}

    # Start input thread
    thread = threading.Thread(target=input_thread, args=(executor, missions, NODE_CLASSES, active_nodes), daemon=True)
    thread.start()

    try:
        executor.spin()
    finally:
        for node in active_nodes.values():
            node.destroy_node()
        heartbeat_node.destroy_node()
        supervisor_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()