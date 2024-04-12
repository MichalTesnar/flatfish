Install I did
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#system-requirements
Starting workflow
```source ~/ros2_humble/install/setup.bash && source ~/ros2_ws/install/setup.bash```
Executing files
```ros2 run mystery ...```
Rebuilding the workspace
```colcon build --packages-select mystery```
Installing dependencies
```rosdep install -i --from-path src --rosdistro humble -y```

# To sync subscription to
```/flatfish/thruster_surge_left/thruster_status```

```/flatfish/thruster_surge_right/thruster_status```

```/flatfish/thruster_sway_front/thruster_status```

```/flatfish/thruster_sway_rear/thruster_status```
all of type 
[flatfish_msgs/msg/ThrusterStatus](https://git.hb.dfki.de/flatfish/drivers/ros2_thruster_enitech/-/blob/master/msg/ThrusterStatus.msg?ref_type=heads)
and ```flatfish/odom_simple/odom``` of type [nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)

# to kill a malfunctioning node running on the system
 killall my_node

# to connect to the system
```ssh flatfish@192.168.128.50 -XC```
```cd flatfish_ws && source install/setup.zsh```

# to start up everything
## Starting up hardware
```cd ~/dfki-software/drivers-efuse_board/build/src```  
```./turnOnThrusters # should be on by default, sometimes also needs a few times```
```./turnOnDVL # might need to be done 3 times, for some reason```
## starting up ROS topics for thrusters
```ros2 launch flatfish_launch launch_canbus.launch.py```
```ros2 launch flatfish_launch launch_thrusters.launch.py```
```ros2 launch flatfish_launch launch_thruster_allocation.launch.py```

if it does not work try ```cansend_deactivate_thrusters.sh``` and try again

# to start odometry node
```ros2 launch flatfish_launch flatfish_base.launch.py```
```ros2 launch flatfish_launch odom_simple.launch.py```
check if ```flatfish/odom``` is active be seing ```ros2 topic list``` if not you can restart the commands above or try ```ros2 lifecycle set /flatfish/pose_estimator activate```


# to see topics locally
type ```export ROS_DOMAIN_ID=10``` on both flatfish and your pc (make sure to turn of WiFi for that, be connected to only one network)

# joystick from my machine
cd foxy_ws/
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch ros2_remote_relay launch_remote_relay_joy.py

# depth control
First launch stuff thrusters in a decoupled mode.

```ros2 launch flatfish_launch launch_thrusters_decoupled.launch.py ```

```ros2 launch flatfish_launch launch_thruster_allocation_decoupled.launch.py ```

Then make sure that on flatfish make sure the odom_simple is running

To launch the pose control on flatfish
ros2 launch flatfish_launch launch_pose_control_depth.launch.py

DO NOT FORGET TO

```export ROS_DOMAIN_ID=10 && source ros2_humble/install/setup.bash```
on your laptop:
```ros2 topic pub -r 5 /cmd_pose geometry_msgs/msg/PoseStamped "{pose: {position: {z: -3.0}}}" # or different depth```

```ros2 topic pub -r 5 /cmd_velocity geometry_msgs/msg/TwistStamped```

then to activate the control
```ros2 lifecycle set /flatfish/auv_pose_control_node activate```

