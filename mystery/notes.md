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


# TODO
- fix importing model from one folder