import os
import yaml
import shutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    try:
        file_path = os.path.join(get_package_share_directory(package_name), file_path)
        with open(file_path, 'r') as file:
            return yaml.safe_load(file) or {}
    except Exception as e:
        print(f"Failed to load YAML file {file_path}: {e}")
        return {}

def load_or_default_yaml(package_name, file_path, default_dict):
    try:
        abs_path = os.path.join(get_package_share_directory(package_name), file_path)
        with open(abs_path, 'r') as f:
            return yaml.safe_load(f) or default_dict
    except Exception:
        print(f"Warning: Could not load {file_path}, using default config.")
        return default_dict

def generate_launch_description():
    try:
        get_package_share_directory("pegasus_arm_description")
    except Exception as e:
        return LaunchDescription([
            LogInfo(msg=f"Error: Package 'pegasus_arm_description' not found: {e}"),
            Shutdown()
        ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory("pegasus_arm_description")
    model_path = os.path.join(pkg_share, "URDF", "pegasus_arm.urdf.xacro")

    srdf_path = os.path.join(pkg_share, "srdf", "pegasus_arm.srdf")
    rviz_config_file = os.path.join(pkg_share, "rviz", "moveit.rviz")
    ros2_controllers_yaml = load_yaml("pegasus_arm_description", "config/ros2_controllers.yaml")
    if not isinstance(ros2_controllers_yaml, dict):
        ros2_controllers_yaml = {}

    if (
        "joint_state_broadcaster" in ros2_controllers_yaml
        and isinstance(ros2_controllers_yaml["joint_state_broadcaster"], dict)
        and "ros__parameters" in ros2_controllers_yaml["joint_state_broadcaster"]
        and ros2_controllers_yaml["joint_state_broadcaster"]["ros__parameters"] is None
    ):
        ros2_controllers_yaml["joint_state_broadcaster"]["ros__parameters"] = {}

    moveit_controllers_yaml_path = os.path.join(pkg_share, "config", "moveit_controllers.yaml")
    kinematics_yaml_path = os.path.join(pkg_share, "config", "kinematics.yaml")

    ompl_planning_default = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': [
                'default_planner_request_adapters/AddTimeParameterization',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ],
            'start_state_max_bounds_error': 0.1
        },
        'planner_configs': {
            'RRTConnect': {
                'type': 'geometric::RRTConnect',
                'range': 0.1,
                'goal_bias': 0.05
            }
        },
        'pegasus_arm': {
            'planner_configs': ['RRTConnect'],
            'projection_evaluator': 'joints(joint1, joint2)',
            'longest_valid_segment_fraction': 0.05
        },
        'planning_plugins': [
            'ompl_interface/OMPLPlanner'
        ]
    }
    ompl_planning_dict = load_or_default_yaml(
        "pegasus_arm_description",
        "config/ompl_planning.yaml",
        ompl_planning_default
    )

    if not os.path.exists(srdf_path):
        return LaunchDescription([
            LogInfo(msg=f"Error: SRDF file not found at {srdf_path}"),
            Shutdown()
        ])
    if not os.path.exists(rviz_config_file):
        return LaunchDescription([
            LogInfo(msg=f"Error: RViz config file not found at {rviz_config_file}"),
            Shutdown()
        ])

    if not os.path.exists(kinematics_yaml_path):
        print(f"Warning: kinematics.yaml not found at {kinematics_yaml_path}, using default.")
        kinematics_yaml = {'arm': {'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin'}}
    else:
        kinematics_yaml = load_yaml("pegasus_arm_description", "config/kinematics.yaml")
        if not isinstance(kinematics_yaml, dict):
            kinematics_yaml = {'arm': {'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin'}}

    moveit_controllers = load_yaml("pegasus_arm_description", "config/moveit_controllers.yaml")
    if not isinstance(moveit_controllers, dict) or not moveit_controllers:
        moveit_controllers = {
            'moveit_simple_controller_manager': ros2_controllers_yaml,
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        }

    print(f"Resolved model path: {model_path}")
    if not os.path.exists(model_path):
        print(f"Error: Xacro file not found at {model_path}")
        return LaunchDescription([
            LogInfo(msg=f"Error: Xacro file not found at {model_path}"),
            Shutdown()
        ])

    xacro_executable = shutil.which('xacro')
    if xacro_executable is None:
        print("Error: 'xacro' executable not found in PATH.")
        return LaunchDescription([
            LogInfo(msg="Error: 'xacro' executable not found in PATH. Please install ros-xacro or add it to your PATH."),
            Shutdown()
        ])

    robot_description_content = Command([
        TextSubstitution(text=xacro_executable + " "),
        model_path
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    with open(srdf_path, 'r') as srdf_file:
        robot_description_semantic_content = srdf_file.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
            {'publish_frequency': 100.0}
        ]
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )


    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
            ros2_controllers_yaml
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '30'],
        output='screen'
    )


    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pegasus_arm_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '30'],
        output='screen'
    )


    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': use_sim_time},
            ompl_planning_dict,
            moveit_controllers
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        LogInfo(msg="Starting Pegasus Arm MoveIt Launch..."),
        rsp_node,
        static_tf_publisher,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node
    ])
