> # rospi-net
> ### v1.0
> A ROS 2 package for stress-testing networking on Raspberry Pi (or other platforms) by creating a configurable router node that publishes and subscribes to multiple topics.

## Features

- **Router Node:** Dynamically creates publishers and subscribers on specified topics for network stress testing.
- **Configurable:** Topics are set via ROS 2 parameters or YAML files.

## Usage

### 1. Build the workspace

```sh
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Example Parameter YAML

Create a file like `src/netstress/params/diff_rate_topics.yaml`:
```yaml
router:
  ros__parameters:
    publish_topics: ["t1", "t2", "t3", "t4", "t5"]
    subscribe_topics: ["t1", "t2", "t3", "t4", "t5"]
    publish_rates: [1.0, 2.0, 3.0]
    default_publish_rate: 1.0
```
You can use some of the ones premade for testing.

### 3. Run the Router Node
```sh
ros2 run netstress router --ros-args --params-file <location>
```

* The node will publish messages on each topic in `talk_topics` and subscribe to each topic in `listen_topics`.
* You can monitor the logs to see message counts.

## Detailed Usage

The `router` node can be configured to publish and subscribe to multiple topics with customizable publish rates. Parameters can be passed via command line or YAML files.

### Example Parameter Configuration

```yaml
router:
  ros__parameters:
    publish_topics: ["t1", "t2", "t3", "t4", "t5"]
    subscribe_topics: ["t1", "t2", "t3", "t4", "t5"]
    publish_rates: [1.0, 2.0, 3.0]                    # in Hz, matched in order to publish_topics
    default_publish_rate: 1.0                         # used if publish_rates list is shorter than publish_topics
```

### Behavior Overview

* The node publishes messages on each topic in `publish_topics` at the specified rates.
* It subscribes to each topic in `subscribe_topics`, logging received messages and total counts.
* Messages sent include an incrementing counter for tracking.
* Currentlly params while running will not affect the node.

### Sample Log Output

```py
[INFO] [1750269604.035013766] [router]: Publish topics: ['t1', 't2', 't3', 't4', 't5']
[INFO] [1750269604.035272333] [router]: Subscribe topics: ['t1', 't2', 't3', 't4', 't5']
[INFO] [1750269604.376975246] [router]: [t3] Published: 'Stress message 0'
[INFO] [1750269604.379816702] [router]: [t3] Received: 'Stress message 0', total count: 1
[INFO] [1750269604.541940348] [router]: [t2] Published: 'Stress message 0'
[INFO] [1750269604.542896030] [router]: [t2] Received: 'Stress message 0', total count: 1
[INFO] [1750269604.708618861] [router]: [t3] Published: 'Stress message 1'
[INFO] [1750269604.709035552] [router]: [t3] Received: 'Stress message 1', total count: 2
[INFO] [1750269605.041722181] [router]: [t1] Published: 'Stress message 0'
[INFO] [1750269605.044445573] [router]: [t2] Published: 'Stress message 1'
[INFO] [1750269605.045742655] [router]: [t3] Published: 'Stress message 2'
[INFO] [1750269605.047011628] [router]: [t4] Published: 'Stress message 0'
[INFO] [1750269605.047616835] [router]: [t5] Published: 'Stress message 0'
[INFO] [1750269605.048189567] [router]: [t1] Received: 'Stress message 0', total count: 1
[INFO] [1750269605.048672653] [router]: [t2] Received: 'Stress message 1', total count: 2
[INFO] [1750269605.049109208] [router]: [t3] Received: 'Stress message 2', total count: 3
[INFO] [1750269605.049772231] [router]: [t4] Received: 'Stress message 0', total count: 1
[INFO] [1750269605.050137232] [router]: [t5] Received: 'Stress message 0', total count: 1
[INFO] [1750269605.375529904] [router]: [t3] Published: 'Stress message 3'
[INFO] [1750269605.376003665] [router]: [t3] Received: 'Stress message 3', total count: 4
[INFO] [1750269605.541756414] [router]: [t2] Published: 'Stress message 2'
[INFO] [1750269605.542146392] [router]: [t2] Received: 'Stress message 2', total count: 3
[INFO] [1750269605.708620050] [router]: [t3] Published: 'Stress message 4'
[INFO] [1750269605.709041267] [router]: [t3] Received: 'Stress message 4', total count: 5
[INFO] [1750269606.041707021] [router]: [t1] Published: 'Stress message 1'
[INFO] [1750269606.045459948] [router]: [t2] Published: 'Stress message 3'
[INFO] [1750269606.047472785] [router]: [t3] Published: 'Stress message 5'
[INFO] [1750269606.047834218] [router]: [t4] Published: 'Stress message 1'
[INFO] [1750269606.048027715] [router]: [t5] Published: 'Stress message 1'
[INFO] [1750269606.048235325] [router]: [t1] Received: 'Stress message 1', total count: 2
[INFO] [1750269606.048429139] [router]: [t2] Received: 'Stress message 3', total count: 4
[INFO] [1750269606.048876667] [router]: [t3] Received: 'Stress message 5', total count: 6
[INFO] [1750269606.049604881] [router]: [t4] Received: 'Stress message 1', total count: 2
[INFO] [1750269606.050002344] [router]: [t5] Received: 'Stress message 1', total count: 2

...
```

### Sending Messages to Listen Topics

You can manually publish messages to a subscribed topic to test receiving:

```bash
ros2 topic pub 't4' std_msgs/String 'data: Test message' -r 4 -t 4
```

## Development Notes

* Only the `router` node is included; standalone listener and talker nodes have been removed.

## References for network analysis

* [ROS 2 Network Analysis on Stack Overflow](https://stackoverflow.com/questions/75967385/ros2-network-analysis)
* [ROS 2: Examine Traffic](https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Examine-Traffic.html)
* [ros2\_tracing GitHub](https://github.com/ros2/ros2_tracing)

### Future Directions

The current setup could be extended to support **virtual components** configured entirely by parameter files. For example:

* Listening to sensor topics like `left_dist_sensor_data` and `right_dist_sensor_data`.
* Publishing aggregated or processed messages (e.g., `object_position`) based on logic combining input topics.
* Introducing random variation in publish timing within configured bounds.

There are also ideas for virtual services that:

* Listen on specific topics.
* Publish results conditionally.
* Return messages based on inputs.

Achieving this modularly would require designing a configuration system capable of defining such behaviors—possibly extending beyond simple YAML to scriptable configurations.
However, it would more accurately represent a ROS-2 Development environment.

Another idea would be to have one of the PI's or an external computer run a simulated robot using something such as Gazebo, and using that to generate the messages and such that we watch.
However this is less abstract and less controllable.
---

*For stress-testing ROS 2 networks and experimenting with topic routing on embedded platforms.*
