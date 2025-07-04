import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    file_path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file) or {}
    except Exception as e:
        print(f"Failed to load YAML file {file_path}: {e}")
        return {}

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(
            get_package_share_directory("pegasus_arm_description"),
            "URDF",
            "pegasus_arm.urdf.xacro"
        ),
        description='Path to robot Xacro file'
    )

    srdf_path = os.path.join(
        get_package_share_directory("pegasus_arm_description"),
        "srdf",
        "pegasus_arm.srdf"
    )
    ros2_controllers_yaml_path = os.path.join(
        get_package_share_directory("pegasus_arm_description"),
        "config",
        "ros2_controllers.yaml"
    )
    controllers_yaml = load_yaml("pegasus_arm_description", "config/controllers.yaml")
    ompl_planning_yaml = load_yaml("pegasus_arm_description", "config/ompl_planning.yaml")

    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    with open(srdf_path, 'r') as srdf_file:
        robot_description_semantic_content = srdf_file.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, robot_description_semantic, {'use_sim_time': use_sim_time}]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, {"use_sim_time": use_sim_time}, ros2_controllers_yaml_path],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pegasus_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_yaml,
            {"use_sim_time": use_sim_time},
            {
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_simple_controller_manager": controllers_yaml,
                "publish_planning_scene": True,
                "robot_model_name": "pegasus_arm",
            }
        ],
        arguments=["--ros-args", "--log-level", "debug"]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock'),
        model_arg,
        LogInfo(msg="Starting Minimal Pegasus Arm MoveIt launch..."),
        rsp_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
    ])