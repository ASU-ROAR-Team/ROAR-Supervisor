from click import command
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
import subprocess
import psutil


processes = {}

class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")
        self.get_logger().info("Supervisor running")

#Input thread for selecting missions 
def input_thread(executor, missions, nodes, active_nodes):
    while True:
        cmd = input(f"\nChoose a mission {list(missions.keys())}, or 'exit': ").strip()
        if cmd == "exit":
            print("Shutting down...")
            rclpy.shutdown()
            break

        elif cmd in missions:
            # Destroy previously active nodes, heartbeats, and resource monitors
            for node_data in active_nodes.values():
                heartbeat = node_data.get('heartbeat')
                process = node_data.get('process')
                
                if heartbeat:
                    executor.remove_node(heartbeat)
                    heartbeat.destroy_node()
                if process:
                    try:
                        process.terminate()
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                    except Exception as e:
                        print(f"Error terminating process: {e}")
            
            active_nodes.clear()
            processes.clear()

                # Launch new processes for this mission
            for node_name in missions[cmd]:
                file_path = nodes.get(node_name)  # in this case, node_classes maps name -> Python file path
                if file_path:
                        try:
                            # Launch the Python file as a separate process
                            proc = subprocess.Popen(file_path, shell=True)                    
                            print(f"Launched process '{node_name}' with PID: {proc.pid}")
                            processes[node_name] = proc

                            # Create unified Monitor (handles both heartbeat and resource monitoring)
                            heartbeat = HeartbeatPublisher(process_name=node_name, message=f"{node_name}", pid= proc.pid, interval=2)
                            executor.add_node(heartbeat)
                            
                            active_nodes[node_name] = {
                                'heartbeat': heartbeat, 
                                'process': proc
                            }
                        except Exception as e:
                            print(f"Error launching {node_name}: {e}")
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

    supervisor_node = Supervisor()
    heartbeat_node = HeartbeatPublisher(process_name="Supervisor", message=f"Supervisor alive", pid= os.getpid(), interval=5)

    executor = MultiThreadedExecutor()
    executor.add_node(supervisor_node)
    executor.add_node(heartbeat_node)

    active_nodes = {}

    # Start input thread
    thread = threading.Thread(target=input_thread, args=(executor, missions, nodes, active_nodes), daemon=True)
    thread.start()

    try:
        executor.spin()
    finally:
        # Clean up all monitors and processes
        for node in active_nodes.values():
            heartbeat_node = node.get('heartbeat')
            process = node.get('process')
            
            if heartbeat_node:
                heartbeat_node.shutdown()
                executor.remove_node(heartbeat_node)
            if process:
                try:
                    process.terminate()
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                except Exception as e:
                    print(f"Error terminating process: {e}")
        
        executor.remove_node(heartbeat_node)
        heartbeat_node.destroy_node()
        supervisor_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()