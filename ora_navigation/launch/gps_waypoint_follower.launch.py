# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav2_bringup")
    gps_wpf_dir = get_package_share_directory("ora_navigation")
    ora_desc_dir = get_package_share_directory("ora_description")
    lp_dir = get_package_share_directory("line_perception")
    launch_dir = os.path.join(gps_wpf_dir, "launch")
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_mapviz = LaunchConfiguration("use_mapviz")

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RVIZ"
    )

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        "use_mapviz", default_value="False", description="Whether to start mapviz"
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ora_desc_dir, 'launch', "display.launch.py"))
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "dual_ekf_navsat.launch.py")
        )
    )

    cmd_vel_remap = SetRemap('/cmd_vel', '/demo/cmd_vel')

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "mapviz.launch.py")),
        condition=IfCondition(use_mapviz),
    )

    line_perception_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lp_dir, "lines.launch.py"))
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # simulator launch
    ld.add_action(gazebo_cmd)

    # robot localization launch
    ld.add_action(robot_localization_cmd)
    
    ld.add_action(cmd_vel_remap)
    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # viz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(line_perception_cmd)

    return ld