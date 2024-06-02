import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ora_integration')
    setup_path = os.path.join(pkg_share, 'setup_can_jetson.sh')

    setup = ExecuteProcess(cmd=['bash', setup_path], output='screen')

    can = Node(
        package='ora_integration',
        executable='ora_integration',
        name='ora_integration',
        output='screen'
    )
    
    w2t = Node(
        package='ora_integration',
        executable='wheel_to_twist',
        name='wheel_to_twist',
        output='screen'
    )

    t2w = Node(
        package='ora_integration',
        executable='twist_to_wheel',
        name='twist_to_wheel',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(setup)
    ld.add_action(can)
    ld.add_action(w2t)
    ld.add_action(t2w)

    return ld