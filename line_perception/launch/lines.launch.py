from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_perception',
            namespace='line_perception',
            executable='line_perception',
            name='line_perception'
        ),
        Node(
            package='depth_image_proc',
            #namespace='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='depth_image_proc',
            remappings=[
                ('image_rect', 'depth_camera/depth/image_raw_filtered'),
                ('camera_info', 'depth_camera/camera_info'),
                ('points', 'lines')
            ]
        )
    ])