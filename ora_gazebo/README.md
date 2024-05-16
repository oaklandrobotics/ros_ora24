# ora_gazebo

This package contains two worlds and a launch file for simulation in Gazebo.

## Usage

Build and source the workspace:

```sh
colcon build --symlink-install
. install/setup.bash
```

Launch Gazebo with a world:

```sh
ros2 launch ora_gazebo gazebo.launch.py world_path:=<absolute/path/to/world/file>
```

## Worlds

The package contains two worlds:

- `igvc_world.sdf`: An example of a typical IGVC course with barrels and lines
- `basic_world.sdf`: A simple world with a ground plane and some primitive objects, like a cube and a sphere

If the `world_path` argument is not specified, the launch file will default to `igvc_world.sdf`.