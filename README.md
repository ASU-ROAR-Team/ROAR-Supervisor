Supervisor package

- `supervisor/supervisor.py`: Supervisor node — loads `config.yaml`, prompts user for a mission, launches/stops nodes and their heartbeats in a shared executor.
- `supervisor/heartbeat_publisher.py`: `HeartbeatPublisher` — publishes periodic heartbeat messages on `/heartbeat` with a unique node name.
- `supervisor/config.yaml`: Configuration — lists `missions` and maps node keys to full import paths (package.module.ClassName).
- `supervisor/my_first_node.py`: Example node — simple node that creates `self.heartbeat` for the supervisor to run.
- `setup.py`: Package manifest — ensures `config.yaml` is installed into the package share so the supervisor can read it.

Quick build & run (copy/paste):

```bash
cd /path/to/your/workspace
colcon build
source install/setup.bash
ros2 run supervisor supervisor
```

For remote team to run all modules:

In another terminal:
```
ros2 topic pub --once /mission_cmd std_msgs/msg/String "{data: '{\"command\":\"start\",\"mission\":\"REMOTE\"}'}"
```

Quick debug commands:

```bash
ros2 topic echo /heartbeat   # view heartbeat messages
ros2 node list               # list nodes in the ROS graph
```

One-line summary:

Supervisor node that runs other nodes based on user input; each node exposes a heartbeat and the supervisor reads configuration from `config.yaml`.
