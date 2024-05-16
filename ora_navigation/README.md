# ora_navigation

For ORA robot navigation (Nav2)

Search TODO to see things that need to be tweaked/modified/fixed/whatever

## Commands 'n' stuff

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
ros2 launch ora_navigation dual_ekf_navsat.launch.py
```

To launch GPS waypoint follower:
```
ros2 launch ora_navigation gps_waypoint_follower.launch.py use_rviz:=True
```

To launch mapviz:
```
ros2 launch ora_navigation mapviz.launch.py
```

## Resources

- https://navigation.ros.org/setup_guides/index.html
- https://navigation.ros.org/tutorials/docs/navigation2_with_gps.html
- https://navigation.ros.org/tutorials/docs/integrating_vio.html

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