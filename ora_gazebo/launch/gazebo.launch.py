import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ora_gazebo').find('ora_gazebo')
    world_path = os.path.join(pkg_share, 'world/igvc_world.sdf')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='world_file', 
                                             default_value=world_path, 
                                             description="Path to world file"),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen')
    ])