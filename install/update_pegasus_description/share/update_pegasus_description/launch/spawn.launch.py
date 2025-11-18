import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_arm_description = get_package_share_directory('update_pegasus_description')
    robot_arm_description_share = get_package_prefix('update_pegasus_description')
    
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=os.path.join(robot_arm_description, 'config', 'pegasus.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )
    
    # Set model path for new Gazebo
    model_path = os.path.join(robot_arm_description, "models")
    model_path += pathsep + os.path.join(robot_arm_description_share, "share")
    env_var = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path)
    
    # Robot description parameter
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Start new Gazebo (gz sim) with default empty world
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4'],  # No world file = default empty world
        output='screen'
    )
    
    # Bridge between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )
    
    # Spawn robot in new Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'pegasus',
            '-topic', '/robot_description'
        ],
        output='screen'
    )
    
    # Register event handler to start spawn after gazebo starts
    spawn_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_gazebo,
            on_start=[bridge, spawn_robot]
        )
    )
    
    return LaunchDescription([
        env_var,
        model_arg,
        robot_state_publisher_node,
        start_gazebo,
        spawn_after_gazebo
    ])
