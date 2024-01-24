from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    EqualsSubstitution,
)
from launch.conditions import IfCondition


"""
Description:
    This launch file launches the d435i
"""


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                ),
                launch_arguments={
                    "camera_name": "d435i",
                    "device_type": "d435i",
                    "pointcloud.enable": "true",
                    "enable_depth": "true",
                    "align_depth.enable": "true",
                    "spatial_filter.enable": "true",
                    "temporal_filter.enable": "true",
                    "decimation_filter.enable": "true",
                    "depth_module.enable_auto_exposure": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("image_proc"), "launch", "image_proc.launch.py"]
                )
            ),
            Node(package="packing",
                 executable="detect_object"),
        ]
    )
