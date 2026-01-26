from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (DeclareLaunchArgument, Shutdown, IncludeLaunchDescription,
                            SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, EqualsSubstitution,
                                  Command, FindExecutable, PythonExpression)
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python import get_package_share_directory
import yaml
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_kinect2", default_value="true",
            description="Use the Kinect2 Camera. If 'false', will attempt to use usb camera or built in webcam"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('kinect2_bridge'),
                    'launch',
                    'kinect2_bridge_launch.yaml'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration("use_kinect2")),
        ),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            condition=UnlessCondition(LaunchConfiguration("use_kinect2")),
            arguments=["-p framerate:=30.0 -p pixel_format:=rgb8"]
        ),
        Node(
            package="handcv",
            executable="handcv",
        ),
    ])
