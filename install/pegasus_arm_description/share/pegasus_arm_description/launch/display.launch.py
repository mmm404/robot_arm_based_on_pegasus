import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='/home/mmms/ros2-ws/Sim/pegasus_arm_description/URDF/my_robot.urdf',
        description='Absolute path to robot URDF file'
    )
    
    srdf_arg = DeclareLaunchArgument(
        'srdf',
        default_value='/path/to/your/pegasus_arm.srdf',  # Replace with actual path to your SRDF
        description='Absolute path to robot SRDF file'
    )
    
    # Path to your custom RViz config file
    rviz_config_path = '/home/mmms/ros2-ws/install/pegasus_arm_moveit_config/share/pegasus_arm_moveit_config/config/moveit.rviz'
    
    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['cat ', LaunchConfiguration('model')])}],
    )
    
    # Joint State Publisher GUI node
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Load the SRDF
    robot_description_semantic = {'robot_description_semantic': Command(['cat ', LaunchConfiguration('srdf')])}
    
    # Planning Scene Monitor Parameters
    moveit_params = {
        'robot_description': Command(['cat ', LaunchConfiguration('model')]),
        'robot_description_semantic': Command(['cat ', LaunchConfiguration('srdf')]),
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # RViz node, using your display.rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'robot_description': Command(['cat ', LaunchConfiguration('model')])},
            robot_description_semantic
        ],
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        model_arg,
        srdf_arg,
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])