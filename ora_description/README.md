# ora_description

For ORA robot simulation / navigation

Following this guide: https://navigation.ros.org/setup_guides/index.html 

Other stuff:
- https://robotics.stackexchange.com/questions/99626/ros2-tf2-authority-undetectable-no-authority-available
- https://answers.ros.org/question/404225/cant-visualize-in-rviz2-with-odom-as-fixed-frame/
- https://automaticaddison.com/set-up-the-odometry-for-a-simulated-mobile-robot-in-ros-2/

## Commands 'n' stuff

All the packages you'll need:
```
sudo apt install ros-$ROS_DISTRO-navigation2 \
ros-$ROS_DISTRO-nav2-bringup \
ros-$ROS_DISTRO-turtlebot3-gazebo \
ros-$ROS_DISTRO-joint-state-publisher-gui \
ros-$ROS_DISTRO-xacro \
ros-$ROS_DISTRO-gazebo-ros-pkgs \
ros-$ROS_DISTRO-robot-localization
```

To make the boi go:
```
ros2 launch ora_description display.launch.py
ros2 topic pub /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```