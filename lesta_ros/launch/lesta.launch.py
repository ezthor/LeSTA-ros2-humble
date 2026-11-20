# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    custom_config = LaunchConfiguration("config").perform(context)
    default_config = Path(get_package_share_directory("lesta")) / "ros2" / f"{mode}.yaml"
    configs = [str(default_config)]
    if custom_config:
        config = Path(custom_config).expanduser().resolve()
        if not config.is_file():
            raise ValueError(f"LeSTA parameter file does not exist: {config}")
        configs.append(str(config))

    overrides = {
        "mode": mode,
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
    }
    model_path = LaunchConfiguration("model_path")
    if model_path.perform(context):
        overrides["model_path"] = ParameterValue(model_path, value_type=str)

    return [
        Node(
            package="lesta",
            executable="lesta_node",
            name="lesta",
            output="screen",
            parameters=[*configs, overrides],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="prediction",
                              choices=["labels", "prediction", "mapping"]),
        DeclareLaunchArgument("config", default_value="", description="Optional ROS 2 parameter overlay"),
        DeclareLaunchArgument("model_path", default_value="", description="TorchScript checkpoint override"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=setup),
    ])
