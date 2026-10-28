"""
Launch Nav2 navigation with the rrt_planner/RRTPlannerROS global planner plugin.

Usage (TurtleBot 3, default):
    ros2 launch rrt_planner rrt_turtlebot.launch.py map:=<path/to/map.yaml>

Usage (TurtleBot 4):
    ros2 launch rrt_planner rrt_turtlebot.launch.py map:=<path/to/map.yaml> turtlebot:=4

Prerequisites:
    - A map produced by Mini-Project 1 (slam_toolbox / cartographer).
    - TurtleBot 3: source /opt/ros/$ROS_DISTRO/setup.bash and set TURTLEBOT3_MODEL.
    - TurtleBot 4: source /opt/ros/$ROS_DISTRO/setup.bash.
    - The rrt_planner package must be built:
        colcon build --packages-select rrt_planner
        source install/setup.bash
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare


def validate_turtlebot_version(context, *args, **kwargs):
    version = context.launch_configurations.get('turtlebot', '3')
    if version not in ('3', '4'):
        raise RuntimeError(
            f"Invalid turtlebot version '{version}'. Must be 3 or 4."
        )
    return []


def generate_launch_description():
    pkg_share = get_package_share_directory('rrt_planner')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    turtlebot_arg = DeclareLaunchArgument(
        'turtlebot',
        default_value='3',
        description='TurtleBot version: 3 or 4',
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='waffle_pi',
        description='TurtleBot 3 model (burger | waffle | waffle_pi) — ignored when turtlebot:=4',
    )

    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to the map YAML file from Mini-Project 1',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    # ---------------------------------------------------------------------------
    # Validate turtlebot version
    # ---------------------------------------------------------------------------
    validate_version = OpaqueFunction(function=validate_turtlebot_version)

    # ---------------------------------------------------------------------------
    # TurtleBot 3 — set TURTLEBOT3_MODEL env var required by turtlebot3 packages
    # ---------------------------------------------------------------------------
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=LaunchConfiguration('model'),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('turtlebot'), "' == '3'"])
        ),
    )

    # ---------------------------------------------------------------------------
    # TurtleBot 3 navigation (nav2_bringup via turtlebot3_navigation2)
    # ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=<map.yaml>
    # ---------------------------------------------------------------------------
    tb3_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot3_navigation2'), '/launch/navigation2.launch.py'
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_share, 'config', 'rrt_planner_nav2_params.yaml'),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('turtlebot'), "' == '3'"])
        ),
    )

    # ---------------------------------------------------------------------------
    # TurtleBot 4 navigation (nav2_bringup via turtlebot4_navigation)
    # ros2 launch turtlebot4_navigation nav2.launch.py map:=<map.yaml>
    # ---------------------------------------------------------------------------
    tb4_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot4_navigation'), '/launch/nav2.launch.py'
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_share, 'config', 'rrt_planner_nav2_params.yaml'),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('turtlebot'), "' == '4'"])
        ),
    )

    return LaunchDescription([
        turtlebot_arg,
        model_arg,
        map_arg,
        use_sim_time_arg,
        validate_version,
        set_tb3_model,
        tb3_nav,
        tb4_nav,
    ])
