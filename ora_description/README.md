# ora_description

For ORA robot simulation

Following this guide: https://navigation.ros.org/setup_guides/index.html
https://robotics.stackexchange.com/questions/99626/ros2-tf2-authority-undetectable-no-authority-available
https://answers.ros.org/question/404225/cant-visualize-in-rviz2-with-odom-as-fixed-frame/
https://automaticaddison.com/set-up-the-odometry-for-a-simulated-mobile-robot-in-ros-2/

## Commands 'n' stuff

All the packages you'll need:
```
sudo apt install ros-$ROS_DISTRO-navigation2 \
ros-$ROS_DISTRO-nav2-bringup \
ros-$ROS_DISTRO-turtlebot3-gazebo \
ros-$ROS_DISTRO-joint-state-publisher-gui \
ros-$ROS_DISTRO-xacro \
ros-$ROS_DISTRO-gazebo-ros-pkgs
```

- `ros2 launch ora_description display.launch.py`