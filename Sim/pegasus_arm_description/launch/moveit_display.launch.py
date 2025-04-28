import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import yaml

# Load the controllers configuration from the YAML file
controllers_file_path = os.path.join(
    get_package_share_directory("pegasus_arm_moveit_config"),
    "config",
    "ros2_controllers.yaml"
)
with open(controllers_file_path, 'r') as file:
    controllers_config = yaml.safe_load(file)

def generate_launch_description():
    # MoveIt configuration
    moveit_config = MoveItConfigsBuilder("pegasus_arm", package_name="pegasus_arm_moveit_config").to_moveit_configs()
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Define path to the URDF file
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='/home/mmms/ros2-ws/Sim/pegasus_arm_description/URDF/my_robot.urdf',
        description='Absolute path to robot URDF file'
    )
    
    # Path to your custom RViz config file
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("pegasus_arm_moveit_config"),
        "config",
        "moveit.rviz"
    ])
    
    # Path to your ros2_controllers.yaml file
    controllers_yaml_path = PathJoinSubstitution([
        FindPackageShare("pegasus_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    ])
    
    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['cat ', LaunchConfiguration('model')]), value_type=str),
            'use_sim_time': use_sim_time,
        }],
    )

    # Controller Manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            controllers_config
        ],
        output="screen"
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Arm Controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pegasus_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            moveit_config.move_group_capabilities,
            {"use_sim_time": use_sim_time},
            {"robot_description_planning.cartesian_limits.max_trans_vel": 0.1},
            {"robot_description_planning.cartesian_limits.max_trans_acc": 0.1},
            {"robot_description_planning.cartesian_limits.max_trans_dec": 0.1},
            {"robot_description_planning.cartesian_limits.max_rot_vel": 0.1},
            {"controller_names": ["pegasus_arm_controller"]},
        ],
    )

    # Joint State Publisher GUI node
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz node with MoveIt configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Static TF node
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "world",
            "--child-frame-id", "base_link"
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        model_arg,
        rsp_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        jsp_gui_node,
        move_group_node,
        static_tf_node,
        rviz_node,
    ])
