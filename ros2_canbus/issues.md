# Issues

## Watch and unwatch

- publish all messages at the same time as an Array (?) 
    + unknown devices publish in one array message (**yes**)
    + the watch has they own publisher created in the struct with desired name! (**yes**)
- Latched topics (?) not to lose message (**nop**): When a connection is latched, the last message published is saved and automatically sent to any future subscribers that connect. 
- Watch certain Ids (given my the static params) (**yes**)
- enable service to add and remove ids (**yes**) or only during reconfiguration (already implemented as it reads the static params again)
<!-- - option to read and write everything, independent of whether they are watched or unwatched -->
- Queue the messages and send them during update and not directly just in case (**yes**)

How is programm
- if unwatched insert it in the watch list
- create more timers for the check interval and statistics interval (**yes**)


## Other issues
- To or 2 conversion libs
- Transformation from rostime to std::chrono
- Make clear difference between time vs can_time vs stamp (?) 
- using ROS time (simulated time) rclcpp::Time::now() or system time rclcpp::Clock for ROS2 std::chrono::system_clock
- static params issue (?)

- is the check errors timer periodic or only in case there is no message since a while? (timeout with reset (?))


## TODO
- Watch and unwatch
    - [ ] with the server 
- Check refracturation 
    - [ ] check every state activate deactivate remove etc.

- use mask (?)


- watch from static params (only during reconfiguration)
- services (on_send)
    + isWatch(id)  bool
    + getTopicName(id) with namespace from node this->get_namespace()
    + watch(id, name)
    + unwatch(id)
- change name setRosNode by setPublishersAndSubscribers (?)

- We are publishing all the time old messages ! need a flag
- we don't unwatch by name
- customize assert (?)
- getTopic: namespace (?)
- automatic topic name (?)
- only services when active (?) or also in configure mode (?)