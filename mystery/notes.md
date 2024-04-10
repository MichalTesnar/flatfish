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

# how to do sync
https://git.hb.dfki.de/deepersense/deepersense-ros/sonavision-ros/-/blob/master/ros2_nightvision_inference/nightvision.py?ref_type=heads

# To sync subscription to
/flatfish/thruster_surge_left/thruster_status
/flatfish/thruster_surge_right/thruster_status
/flatfish/thruster_sway_front/thruster_status
/flatfish/thruster_sway_rear/thruster_status

all of type

flatfish_msgs/msg/ThrusterStatus (https://git.hb.dfki.de/flatfish/drivers/ros2_thruster_enitech/-/blob/master/msg/ThrusterStatus.msg?ref_type=heads)

and

flatfish/odom

of type

nav_msgs/msg/Odometry (https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)

ssh flatfish@192.168.128.50 -XC
cd flatfish_ws && source install/setup.zsh

#to kill a node
      killall my_node

# to start odometry node
cd ~/dfki-software/drivers-efuse_board/build/src 
./turnOnDVL
cd ~/flatfish_ws
ros2 launch flatfish_launch flatfish_base.launch.py
ros2 launch flatfish_launch poseukf_node.launch.py
ros2 lifecycle set /flatfish/pose_estimator activate


