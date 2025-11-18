#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('update_pegasus_description')
    moveit_config_pkg = FindPackageShare('your_moveit_config_package')  # Replace with your actual package name
    
    # File paths
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'pegasus.urdf.xacro'])
    srdf_file = PathJoinSubstitution([moveit_config_pkg, 'config', 'pegasus.srdf'])
    kinematics_file = PathJoinSubstitution([moveit_config_pkg, 'config', 'kinematics.yaml'])
    joint_limits_file = PathJoinSubstitution([moveit_config_pkg, 'config', 'joint_limits.yaml'])
    ompl_planning_file = PathJoinSubstitution([moveit_config_pkg, 'config', 'ompl_planning.yaml'])
    moveit_controllers_file = PathJoinSubstitution([moveit_config_pkg, 'config', 'moveit_controllers.yaml'])
    ros2_controllers_file = PathJoinSubstitution([pkg_share, 'config', 'ros2_controllers.yaml'])
    
    # Robot description
    robot_description = {'robot_description': urdf_file}
    
    # Robot semantic description
    robot_description_semantic = {'robot_description_semantic': srdf_file}
    
    # Kinematics configuration
    kinematics_config = PathJoinSubstitution([moveit_config_pkg, 'config', 'kinematics.yaml'])
    
    # Planning configuration
    planning_config = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'planning_adapters': """default_planning_request_adapters/AddTimeOptimalParameterization 
                               default_planning_request_adapters/FixWorkspaceBounds 
                               default_planning_request_adapters/FixStartStateBounds 
                               default_planning_request_adapters/FixStartStateCollision 
                               default_planning_request_adapters/FixStartStatePathConstraints""",
        'request_adapters': """default_planning_request_adapters/AddTimeOptimalParameterization 
                              default_planning_request_adapters/FixWorkspaceBounds 
                              default_planning_request_adapters/FixStartStateBounds 
                              default_planning_request_adapters/FixStartStateCollision 
                              default_planning_request_adapters/FixStartStatePathConstraints""",
        'start_state_max_bounds_error': 0.1,
    }
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        # Static Transform Publisher (world to base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='log',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
        ),
        
        # ros2_control node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, ros2_controllers_file],
            output='both',
        ),
        
        # Controller spawners
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['pegasus_arm_controller'],
            output='screen',
        ),
        
        # MoveGroup node
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_config,
                planning_config,
                joint_limits_file,
                moveit_controllers_file,
            ],
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', PathJoinSubstitution([moveit_config_pkg, 'config', 'moveit.rviz'])],
            parameters=[
                robot_description,
                robot_description_semantic,
                planning_config,
                kinematics_config,
            ],
        ),
    ])
