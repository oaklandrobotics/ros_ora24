# ora_description

For ORA robot simulation / navigation

Search TODO to see things that need to be tweaked/modified/fixed/whatever

## Resources

- https://navigation.ros.org/setup_guides/index.html
- https://navigation.ros.org/tutorials/docs/navigation2_with_gps.html
- https://navigation.ros.org/tutorials/docs/integrating_vio.html

## Commands 'n' stuff

All the packages you'll need:
```
sudo apt install ros-$ROS_DISTRO-navigation2 \
ros-$ROS_DISTRO-nav2-bringup \
ros-$ROS_DISTRO-turtlebot3-gazebo \
ros-$ROS_DISTRO-joint-state-publisher-gui \
ros-$ROS_DISTRO-xacro \
ros-$ROS_DISTRO-gazebo-ros-pkgs \
ros-$ROS_DISTRO-robot-localization \
ros-$ROS_DISTRO-slam-toolbox \
ros-$ROS_DISTRO-depthimage-to-laserscan \
ros-$ROS_DISTRO-robot-localization \
ros-$ROS_DISTRO-mapviz \
ros-$ROS_DISTRO-mapviz-plugins \
ros-$ROS_DISTRO-tile-map
```

To launch Gazebo and robot description:
```
ros2 launch ora_description display.launch.py
```

To make the robot move, spin, etc:
```
ros2 topic pub --once /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

To launch Nav2 (normally):
```
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/dan/ros2_ws/src/ora_description/config/nav2_params.yaml use_sim_time:=True
```

Temp map -> odom transform:
```
ros2 run tf2_ros static_transform_publisher 2 2 0 0 0 0 map odom --ros-args --remap use_sim_time:=True
```

To launch navsat_transform, and dual-EKFs:
```
ros2 launch ora_description dual_ekf_navsat.launch.py
```

To launch GPS waypoint follower:
```
ros2 launch ora_description gps_waypoint_follower.launch.py use_rviz:=True
```

To launch mapviz:
```
ros2 launch ora_description mapviz.launch.py
```

## Random notes

Concept footprint and radius params:
```yaml
footprint: "[ [0.36, 0.26], [0.36, -0.26], [-0.36, -0.26], [-0.36, 0.26] ]"
robot_radius: 0.5
```

Horizon footprint and radius params:
```yaml
footprint: "[ [0.483, 0.33],
            [0.127, 0.33], 
            [0.127, 0.381], 
            [-0.127, 0.381],
            [-0.127, 0.33],
            [-0.483, 0.33],
            [-0.483, -0.33],
            [-0.127, -0.33],
            [-0.127, -0.381],
            [0.127, -0.381],
            [0.127, -0.33],
            [0.483, -0.33] ]"
robot_radius: 0.5
```