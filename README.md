# ros_ora24

The ORA software for 2024.

## To work on the code

### 1 - Clone the repo
```
mkdir ros2_ws && cd ros2_ws
git clone https://github.com/oaklandrobotics/ros_ora24 src
```

### 2 - Write some code!

This can be done in your IDE of choice, such as VS Code.

### 3 - Build the workspace
```
colcon build --symlink install
. install/setup.bash
```

## To set up the dev environment

- [Install ROS2 Humble on Ubuntu 22.04.](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) After installing, run this command:
  ```
  echo "source /opt/ros/humble/setup.bash" >> .bashrc`
  ```
  (This will automatically run the `source` command with every new terminal session)
- Make sure colcon, pip, and rosdep are installed:
  ```
  sudo apt install python3-colcon-common-extensions python3-pip python3-rosdep2
  ```
- Install the ROS2 package dependencies with `rosdep install --from-paths src --ignore-src`
  - Optionally, use the `-r` flag to skip packages that did not install successfully (such as on the Jetson)
- Install the python dependencies with `pip install -r requirements.txt`