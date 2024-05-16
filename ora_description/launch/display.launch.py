import launch
import launch.launch_description_sources
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ora_description')
    default_model_path = os.path.join(pkg_share, 'src/description/horizon.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            { 'robot_description': Command(['xacro ', LaunchConfiguration('model')]) },
            { 'use_sim_time': LaunchConfiguration('use_sim_time' ) }
        ]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        parameters=[{ 'use_sim_time': LaunchConfiguration('use_sim_time' ) }]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{ 'use_sim_time': LaunchConfiguration('use_sim_time' ) }]
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'horizon', '-topic', 'robot_description', '-timeout', '60', '-x', '-14.547985', '-y', '0.642267', '-z', '0.150999', '-Y', '1.57'],
        output='screen',
    )
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ora_gazebo'), 'launch', 'gazebo.launch.py')
        )
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        #joint_state_publisher_node, #disable this since joints are controlled by gazebo instead of this
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        #robot_localization_node,
        rviz_node
    ])