# ora_description

URDF files for ORA robots

## Usage

Build and source the workspace:
```
colcon build --symlink-install
. install/setup.bash
```

To launch Gazebo and robot description:
```
ros2 launch ora_description display.launch.py
```

## URDF Files

- `concept.urdf`: Basic robot with diff drive wheels, and all sensors
- `horizon.urdf`: Representation of Horzion, with more accurate dimensions and sensor tower
- `horizon_zed.urdf`: Same as `horizon.urdf`, but with ZED URDF instead of generic depth camera