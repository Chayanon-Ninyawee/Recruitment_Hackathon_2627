"""Start your driver with the parameters from config/driver_params.yaml.

    ros2 launch team_driver driver.launch.py

The simulator is separate: `ros2 launch roboracer_referee simulator.launch.py`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    team_driver_dir = get_package_share_directory("team_driver")
    referee_dir = get_package_share_directory("roboracer_referee")

    default_params = os.path.join(
        team_driver_dir,
        "config",
        "driver_params.yaml",
    )

    rviz_config = os.path.join(
        team_driver_dir,
        "config",
        "driver.rviz",
    )

    simulator_launch = os.path.join(
        referee_dir,
        "launch",
        "simulator.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value=default_params,
                description="Parameter YAML for the driver node",
            ),
            # Start simulator without its protected race.rviz
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulator_launch),
                launch_arguments={
                    "rviz": "false",
                }.items(),
            ),
            # Start our driver
            Node(
                package="team_driver",
                executable="driver",
                name="driver",
                output="screen",
                emulate_tty=True,
                prefix=os.path.join(
                    team_driver_dir,
                    "launch",
                    "launch_driver_tmux.sh",
                ),
                parameters=[LaunchConfiguration("params")],
            ),
            # Start our own RViz configuration
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    f"rviz2 -d '{rviz_config}' >/dev/null 2>&1",
                ],
            ),
        ]
    )
