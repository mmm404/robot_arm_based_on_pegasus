import os
import yaml
import logging

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger("pegasus_arm_launch")
logging.basicConfig(level=logging.INFO)

def load_yaml(package_name, relative_path):
    abs_path = os.path.join(get_package_share_directory(package_name), relative_path)
    try:
        with open(abs_path, 'r') as file:
            logger.info(f"Loaded YAML: {abs_path}")
            return yaml.safe_load(file)
    except Exception as e:
        logger.error(f"Failed to load YAML: {abs_path} — {str(e)}")
        return {}

def load_file(package_name, relative_path):
    abs_path = os.path.join(get_package_share_directory(package_name), relative_path)
    try:
        with open(abs_path, 'r') as file:
            logger.info(f"Loaded file: {abs_path}")
            return file.read()
    except Exception as e:
        logger.error(f"Failed to load file: {abs_path} — {str(e)}")
        return ""

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Load robot description files
    robot_description_content = load_file("pegasus_arm_description", "URDF/my_robot.urdf")
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = load_file("pegasus_arm_moveit", "config/pegasus_arm.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # Load YAML configuration files
    kinematics_yaml = {"robot_description_kinematics": load_yaml("pegasus_arm_moveit", "config/kinematics.yaml")}
    joint_limits_yaml = {"robot_description_planning": load_yaml("pegasus_arm_moveit", "config/joint_limits.yaml")}
    ompl_yaml = load_yaml("pegasus_arm_moveit", "config/ompl_planning.yaml")
    
    # Load controllers configuration directly instead of using PathJoinSubstitution
    controllers_yaml = load_yaml("pegasus_arm_moveit", "config/moveit_controllers.yaml")
    logger.info(f"Controllers YAML content: {controllers_yaml}")  # Debug the loaded controllers

    # Planning pipeline config - moved parameters to top level for direct access
    planning_pipeline_config = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": (
            "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints"
        ),
        "start_state_max_bounds_error": 0.1,
        "planning_pipelines": ["ompl"],
        "move_group_capabilities": (
            "move_group/MoveGroupCartesianPathService "
            "move_group/MoveGroupExecuteTrajectoryAction "
            "move_group/MoveGroupKinematicsService "
            "move_group/MoveGroupMoveAction "
            "move_group/MoveGroupPickPlaceAction "
            "move_group/MoveGroupPlanService "
            "move_group/MoveGroupQueryPlannersService "
            "move_group/MoveGroupStateValidationService "
            "move_group/MoveGroupGetPlanningSceneService "
            "move_group/ClearOctomapService"
        ),
        "ompl": ompl_yaml,
    }

    # Trajectory execution parameters
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Explicitly specify the controller manager
    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }

    # Flattened controller_names and controller configuration for direct access
    controller_names = {"controller_names": ["pegasus_arm_controller"]}
    
    controller_config = {
        "pegasus_arm_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [
                "joint1_base",
                "joint2_shoulder",
                "joint3_elbow",
                "joint4",
                "joint5_wrist"
            ]
        }
    }

    rviz_config_file = os.path.join(
        get_package_share_directory("pegasus_arm_moveit"),
        "config",
        "moveit.rviz"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time if true"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],  # Added robot_description
        ),

        # Modified move_group node with flattened controller configuration
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                joint_limits_yaml,
                planning_pipeline_config,
                trajectory_execution,
                moveit_controller_manager,
                controllers_yaml,
                controller_names,           # Explicitly add controller_names
                controller_config,          # Explicitly add controller configuration
                {"use_sim_time": use_sim_time},
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else [],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,           # Added kinematics for RViz
                planning_pipeline_config,   # Added planning config for RViz
                {"use_sim_time": use_sim_time}
            ],
        )
    ])