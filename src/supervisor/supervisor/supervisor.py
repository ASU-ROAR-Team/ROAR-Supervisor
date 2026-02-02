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


processes = {}
log_dir = "supervisor_logs"

class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")

        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_files = {}
        self.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            1000
        )

        self.get_logger().info("Supervisor running")
        
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
    processes[node_name] = proc

    monitor = MonitorNode(process_name=node_name, pid=proc.pid, interval=interval)
    executor.add_node(monitor)

    return {"monitor": monitor, "process": proc}


def stop_node(node_data: dict):
    """Stop a node's process and destroy its monitor if present."""
    monitor = node_data.get("monitor")
    process = node_data.get("process")

    if monitor:
        try:
            monitor.destroy_node()
        except Exception:
            pass
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
    processes.clear()


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


def input_thread(executor: MultiThreadedExecutor, missions: dict, nodes: dict, active_nodes: dict):
    """Thread that accepts user commands to switch missions or query status."""
    while True:
        cmd = input(f"\nChoose a mission {list(missions.keys())}, 'status', or 'exit': ").strip()
        if cmd == "exit":
            print("Shutting down...")
            stop_all(active_nodes, executor)
            rclpy.shutdown()
            break
        if cmd == "status":
            print_status(active_nodes)
            continue
        if cmd not in missions:
            print("Unknown mission")
            continue

        # Switch missions: stop currently running processes/monitors
        stop_all(active_nodes, executor)

        # Launch new processes for selected mission
        for node_name in missions[cmd]:
            file_path = nodes.get(node_name)
            if not file_path:
                print(f"No command configured for node '{node_name}'")
                continue
            try:
                node_info = start_process(node_name, file_path, executor)
                active_nodes[node_name] = node_info
            except Exception as e:
                print(f"Error launching {node_name}: {e}")

        print(f"Mission '{cmd}' launched: {missions[cmd]}")


def main(args=None):
    rclpy.init(args=args)

    missions, nodes = load_config("supervisor", "config.yaml")

    supervisor_node = Supervisor()
    supervisor_monitor = MonitorNode(process_name="Supervisor", pid=os.getpid(), interval=5)

    executor = MultiThreadedExecutor()
    executor.add_node(supervisor_node)
    executor.add_node(supervisor_monitor)

    active_nodes = {}

    # Start input thread
    thread = threading.Thread(target=input_thread, args=(executor, missions, nodes, active_nodes), daemon=True)
    thread.start()

    try:
        executor.spin()
    finally:
        # Stop everything
        stop_all(active_nodes, executor)
        try:
            executor.remove_node(supervisor_monitor)
            supervisor_monitor.destroy_node()
        except Exception:
            pass
        supervisor_node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            # already shutdown from input thread
            pass


if __name__ == "__main__":
    main()