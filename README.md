# rospi-net

A ROS 2 package for stress-testing networking on Raspberry Pi (or other platforms) by creating a configurable router node that publishes and subscribes to multiple topics.

## Features

- **Router Node:** Dynamically creates publishers and subscribers on specified topics for network stress testing.
- **Configurable:** Topics are set via ROS 2 parameters or YAML files.

## Usage

### 1. Build the workspace

```sh
colcon build --symlink-install
source install/setup.bash
```

### 2. Example Parameter YAML

Create a file like `src/netstress/params/test_params.yaml`:

```yaml
router:
  ros__parameters:
    talk_topics: ["topic1", "topic2"]
    listen_topics: ["topic1", "topic3"]
```

### 3. Run the Router Node

```sh
ros2 run netstress router --ros-args --params-file src/netstress/params/test_params.yaml
```

- The node will publish messages on each topic in `talk_topics` and subscribe to each topic in `listen_topics`.
- You can monitor the logs to see message counts.

## Development Notes

- Only the `router` node is included; standalone listener and talker nodes have been removed.
- See the code for how to extend or modify topic behavior.

---

## References & Future Ideas

- [ROS 2 Network Analysis on Stack Overflow](https://stackoverflow.com/questions/75967385/ros2-network-analysis)
- [ROS 2: Examine Traffic](https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Examine-Traffic.html)
- [ros2_tracing GitHub](https://github.com/ros2/ros2_tracing)

---

*For stress-testing ROS 2 networks and experimenting with topic routing on embedded platforms.*
