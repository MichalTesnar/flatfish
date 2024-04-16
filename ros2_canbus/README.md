# Module Map Provider

<!--- protected region package header begins -->
**DISCLAIMER**:
This ROS2 node is based on the [drivers-orogen-canbus](https://github.com/rock-drivers/drivers-orogen-canbus).

**Author:**
  - Miguel Bande Firvida <miguel.bande_firvida@dfki.de>

**Affiliation:** DFKI

**Maintainer:**
  - Miguel Bande Firvida <miguel.bande_firvida@dfki.de>
<!--- protected region package header ends -->

This ROS2 driver to communicate with a CAN Bus.


## Introduction

Ths driver reads and writes a can bus connected to the system. The node is programed as [LifecycleNode](https://index.ros.org/p/lifecycle/github-ros2-demos/).

_IMPORTANT:_ the node allows to watch certain devices connected to the can bus. The devices to be watched can be specified either statically by setting the static parameters (e.g. in the `config/examples.py`) or dynamically by calling a service (see _Services_ below). When a devices is watched, the node will dynamically generate a publisher and subscription for it, so that the user (e.g. a node) can use this interface to exchange messages. You can always use the static interfaces as well (i.e. `<node_name>/rx_unwatched_messages` to read all the read messages in the can bus and `<node_name>/tx_message` to sent one message or `<node_name>/tx_messages` to send multiple messages).


### Quickstart

In order to run the node, follow the next steps:

0. Clone all the repositories required and build workspace:
- https://git.hb.dfki.de/flatfish/drivers/ros2_driver_base
- https://git.hb.dfki.de/flatfish/drivers/driver_canbus
- https://git.hb.dfki.de/flatfish/drivers/ros2_canbus

1. Configure the nodes to be watched by modifying `config/examples.py`.

_NOTE:_ the node will dynamically generate publishers and subscriptions for all the devices with the given can ids, so that the user can use this interface for the watched device. You can always use the static interfaces (i.e. `<node_name>/rx_unwatched_messages` and `<node_name>/tx_message` for one message to be sent or `<node_name>/tx_messages` for multiple messages).
1. Run the following launch files:
```
ros2 launch ros2_canbus_launch.py
```
3. Have fun!


### Interface

The node has it own *C++ library* independent of ROS, which can be then use in another implementation.

#### Parameters


| Parameter      | Type   | Default    | Description           |
| -------------- | ------ | ---------- | --------------------- |
| `device` | string | 'can1' | Can Port |
| `device_type` | int | 0 | enum with the device type (see `Message.hpp` from `driver_can` |
| `check_bus_ok_interval` | double | 0.1 | Time interval in seconds to check if bus has errors |
| `stats_interval` | double | 1.0 | Time interval in seconds to publish statistics |
| `system_frequency` | double | 100.0 | Frequency of the update system during active |
| `watch_devices.can_id` | std::vector<int64>  | list of can ids to be watched |
| `watch_devices.name` | std::vector<std::string> | list of topic names for the corresponding watched devices |

_NOTE:_ the given can ids and names will correspond to the order. Therefore, they must have the same length.
<!--

#### Static YAML Parameters

| Parameter      | Type   | Default    | Description           |
| -------------- | ------ | ---------- | --------------------- |


#### Dynamic Parameters

| Parameter | Type | Range | Default | Description |
| --------- | ---- | ------- | ------- | ----------- | -->


#### Published Topics

| Publisher | Type | Description |
| --------- | ---- | ----------- |
| `<node_name>/rx_unwatched_messages` | `ros2_canbus::msg::ArrayCanbusMessage`| received messages from devices that are not watched |
| `<node_name>/statistics` | `ros2_canbus::msg::CanbusStatistics`| statistics of the communication with the can port |
| `<node_name>/<watch_device.name>/output` | `ros2_canbus::msg::CanbusMessage` | received message from the corresponding watched device |  

#### Subscribed Topics


| Subscriber | Type | Timeout | Description |
| ---------- | ---- | ------- | ----------- |
| `<node_name>/tx_message` | `ros2_canbus::msg::CanbusMessage` | | one message to be sent into can port |
| `<node_name>/tx_messages` | `ros2_canbus::msg::ArrayCanbusMessage` | | multiple message to be sent into can port |
| `<node_name>/<watch_device.name>/input` | `ros2_canbus::msg::CanbusMessage` | | message from the corresponding watched device to be sent into can port|  

#### Services

| Service Server | Type | Description |
| -------------- | ---- | ----------- |
| `<node_name>/watch` | `ros2_canbus::srv::Watch` | Service to dynamically watch new devices |
| `<node_name>/unwatch` | `ros2_canbus::srv::Unwatch` | Service to dynamically unwatch devices that are current watched |
| `<node_name>/get_topic_name` | `ros2_canbus::srv::GetTopicName` | Service to get the topic name of a current watched device |
| `<node_name>/is_watched` | `ros2_canbus::srv::IsWatched` | Service to check whether a device with a given id is currently being watched |


<!-- #### Action
_NOTE:_ all ros interfaces has the following construction `<node_name>/<interface_name>`

| Action Server | Type | Description |
| ------------- | ---- | ----------- | -->
