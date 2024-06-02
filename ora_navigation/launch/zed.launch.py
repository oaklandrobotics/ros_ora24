import launch, os
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ora_nav = get_package_share_directory('ora_navigation')
    zed_share = get_package_share_directory('zed_wrapper')

    params = os.path.join(ora_nav, 'config', 'zed.yaml')

    print(params)
    zed = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(zed_share, 'launch', 'zed_camera.launch.py'),
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_tf': 'false'
        }.items()
    )

    ld = launch.LaunchDescription()

    ld.add_action(zed)

    return ld