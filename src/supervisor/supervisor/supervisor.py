from std_msgs.msg import String
from click import command
import rclpy
import threading
import yaml
import os
import subprocess
import signal
import psutil
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from supervisor.MonitorNode import MonitorNode
from rcl_interfaces.msg import Log

# Directory to store log files
log_dir = "supervisor_logs"

class Supervisor(Node):
    def __init__(self, executor):
        super().__init__("supervisor")
        
        self._executor = executor
        self.missions, self.nodes = load_config("supervisor", "config.yaml")
        self.active_nodes = {}
        self.log_dir = log_dir
        self.log_files = {}

        os.makedirs(self.log_dir, exist_ok=True)
        self.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            1000
        )
        self.create_subscription(
            String,
            '/missions',
            self.mission_control_callback,
            1000
        )

        self.get_logger().info("Supervisor running")
    
    def mission_control_callback(self, msg: String):
        """Callback to handle mission commands and manage active nodes."""
        command = msg.data.strip()
        if command == "status":
            print_status(self.active_nodes)
            return

        if command not in self.missions:
            self.get_logger().error(f"Unknown mission: {command}")
            return

        # Stop all currently running processes
        stop_all(self.active_nodes, self._executor)

        # Start new processes for the selected mission
        for node_name in self.missions[command]:
            file_path = self.nodes.get(node_name)
            if not file_path:
                self.get_logger().error(f"No command configured for node '{node_name}'")
                continue
            try:
                node_info = start_process(node_name, file_path, self._executor)
                self.active_nodes[node_name] = node_info
            except Exception as e:
                self.get_logger().error(f"Error launching {node_name}: {e}")

        self.get_logger().info(f"Mission '{command}' launched: {self.missions[command]}")
        
    def log_callback(self, msg: Log):
        """Callback to log messages from all nodes into separate files."""
        log_file = self.get_log_file(msg.name)

        level = self.level_to_string(msg.level)

        file = os.path.basename(msg.file)
        
        log_file.write(
            f"[{msg.stamp.sec}.{msg.stamp.nanosec:09d}] "
            f"[{level}] "
            f"[{msg.name}::{msg.function} @ {file}:{msg.line}] "
            f"{msg.msg}\n"
        )
        log_file.flush()

    def get_log_file(self, node_name: str):
        """Get or create a log file for the given node name."""
        if node_name not in self.log_files:
            path = os.path.join(self.log_dir, f"{node_name}.log")
            self.log_files[node_name] = open(path, "a")
        return self.log_files[node_name]
    
    def level_to_string(self, level: int) -> str:
        """Convert log level integer to string."""
        if level == 10:
            return "DEBUG"
        elif level == 20:
            return "INFO"
        elif level == 30:
            return "WARN"
        elif level == 40:
            return "ERROR"
        elif level == 50:
            return "FATAL"
        else:
            return "UNKNOWN"

def load_config(package_name: str, file_name: str = "config.yaml"):
    """Load missions and nodes mapping from package share config file."""
    config_path = os.path.join(get_package_share_directory(package_name), file_name)
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    missions = config.get("missions", {})
    nodes = config.get("nodes", {})
    return missions, nodes

def start_process(node_name: str, cmd: str, executor: MultiThreadedExecutor, interval: float = 2.0):
    """Start an external process and attach a HeartbeatPublisher node to it.

    Returns a dict containing the process and monitor.
    """
    # Start the process in its own process group so we can cleanly kill it and any children.
    proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid) ######################################### CHECK FOR SHELL=TRUE ISSUES
    print(f"Launched process '{node_name}' with PID: {proc.pid}")

    monitor = MonitorNode(process_name=node_name, pid=proc.pid, interval=interval)
    executor.add_node(monitor)

    return {"monitor": monitor, "process": proc}


def stop_node(node_data: dict):
    """Stop a node's process and destroy its monitor if present."""
    process = node_data.get("process")
    monitor = node_data.get("monitor")

    if process:
        try:
            # Terminate the whole process group
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception:
                process.terminate()
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                process.kill()
        except Exception as e:
            print(f"Error terminating process: {e}")

    if monitor:
        try:
            monitor.destroy_node()
        except Exception:
            pass


def stop_all(active_nodes: dict, executor: MultiThreadedExecutor):
    """Gracefully stop and remove all active nodes and their processes."""
    for node_name, node_data in list(active_nodes.items()):
        monitor = node_data.get("monitor")
        process = node_data.get("process")
        if monitor:
            try:
                executor.remove_node(monitor)
            except Exception:
                pass
        # stop_node will destroy the monitor and terminate the process
        stop_node(node_data)

    active_nodes.clear()

def print_status(active_nodes: dict):
    """Print current status of all monitored processes."""
    print("\n" + "=" * 80)
    print(f"{'Process':<20} {'PID':<10} {'Status':<15} {'CPU %':<10} {'Memory MB':<12}")
    print("=" * 80)
    for node_name, node_data in active_nodes.items():
        process = node_data.get("process")
        if process:
            try:
                proc = psutil.Process(process.pid)
                status = proc.status()
                cpu = proc.cpu_percent(interval=None)
                mem = proc.memory_info().rss / (1024 * 1024)
                print(f"{node_name:<20} {process.pid:<10} {status:<15} {cpu:<10.2f} {mem:<12.2f}")
            except psutil.NoSuchProcess:
                print(f"{node_name:<20} {process.pid:<10} {'DEAD':<15} {'N/A':<10} {'N/A':<12}")
            except Exception as e:
                print(f"{node_name:<20} {process.pid:<10} {'ERROR':<15} {str(e):<10}")
    print("=" * 80 + "\n")


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    supervisor_node = Supervisor(executor)
    supervisor_monitor = MonitorNode(process_name="Supervisor", pid=os.getpid(), interval=5)

    executor.add_node(supervisor_node)
    executor.add_node(supervisor_monitor)

    try:
        executor.spin()
    finally:
        # Stop everything
        stop_all(supervisor_node.active_nodes, executor)
        try:
            executor.remove_node(supervisor_monitor)
            supervisor_monitor.destroy_node()
        except Exception:
            pass
        supervisor_node.destroy_node()
        for f in supervisor_node.log_files.values():
            try:
                f.close()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except RuntimeError:
            # already shutdown from input thread
            pass


if __name__ == "__main__":
    main()