from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "pegasus", package_name="update_pegasus_description"
    ).to_moveit_configs()

    # Path to your Xacro
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare("update_pegasus_description"),
        "config",
        "pegasus.urdf.xacro"
    ])

    # robot_state_publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_xacro])
        }]
    )

    # joint_state_publisher (connects to same robot_description)
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "use_gui": True,
            "robot_description": Command(["xacro ", urdf_xacro])
        }]
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        generate_move_group_launch(moveit_config)
    ])

