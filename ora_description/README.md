# ora_description

For ORA robot simulation / navigation

Following this guide: https://navigation.ros.org/setup_guides/index.html 

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
ros-$ROS_DISTRO-depthimage-to-laserscan
```

To make the boi go:
```
ros2 launch ora_description display.launch.py
ros2 topic pub --once /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### random notes

`footprint: "[ [0.36, 0.26], [0.36, -0.26], [-0.36, -0.26], [-0.36, 0.26] ]"`

`robot_radius: 0.5`

`ros2 run tf2_ros static_transform_publisher 2 2 0 0 0 0 map odom --ros-args --remap use_sim_time:=True`
`ros2 launch nav2_bringup navigation_launch.py params_file:=/home/dan/ros2_ws/src/ora_description/config/nav2_params.yaml use_sim_time:=True`
