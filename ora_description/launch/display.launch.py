import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ora_description').find('ora_description')
    default_model_path = os.path.join(pkg_share, 'src/description/concept.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz') #this is the default one
    #default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config_with_maps.rviz')
    #default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config_with_scan.rviz')
    world_path = os.path.join(pkg_share, 'world/sim_world.sdf')

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
        arguments=['-entity', 'concept', '-topic', 'robot_description', '-timeout', '60', '-x', '-14.547985', '-y', '0.642267', '-z', '0.150999', '-Y', '1.57'],
        output='screen',
    )
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    depthimage_to_laserscan_node = launch_ros.actions.Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/di2ls.yaml')],
        remappings=[
            ('/depth_camera_info', '/depth_camera/depth/camera_info'),
            ('/depth', '/depth_camera/depth/image_raw')
            ],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        #depthimage_to_laserscan_node,
        #joint_state_publisher_node, #disable this since joints are controlled by gazebo instead of this
        robot_state_publisher_node,
        spawn_entity,
        #robot_localization_node,
        rviz_node
    ])