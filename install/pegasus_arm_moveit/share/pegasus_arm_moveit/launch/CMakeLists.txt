import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def load_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        print(f"ERROR: Could not read file: {file_path}")
        return None

def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        print(f"ERROR: Could not read yaml file: {file_path}")
        return None

def generate_launch_description():
    # Define actual file paths in your workspace
    urdf_file = "/home/mmms/ros2-ws/Sim/pegasus_arm_description/URDF/my_robot.urdf"
    srdf_file = "/home/mmms/ros2-ws/src/pegasus_arm_moveit/moveit_srdf/config/pegasus_arm.srdf"
    joint_limits_file = "/home/mmms/ros2-ws/src/pegasus_arm_moveit/moveit_srdf/config/joint_limits.yaml"
    kinematics_file = "/home/mmms/ros2-ws/src/pegasus_arm_moveit/moveit_srdf/config/kinematics.yaml"
    
    # Ensure URDF file exists
    if not os.path.exists(urdf_file):
        print(f"ERROR: URDF file not found at {urdf_file}. Please provide the correct file.")
        return LaunchDescription()  # Return an empty launch description to prevent further errors.
    
    # Ensure SRDF file exists
    if not os.path.exists(srdf_file):
        print(f"ERROR: SRDF file not found at {srdf_file}. Please provide the correct file.")
        return LaunchDescription()
    
    # Create kinematics.yaml if it doesn't exist
    if not os.path.exists(kinematics_file):
        os.makedirs(os.path.dirname(kinematics_file), exist_ok=True)
        with open(kinematics_file, 'w') as f:
            yaml.dump({
                'manipulator': {
                    'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                    'kinematics_solver_search_resolution': 0.005,
                    'kinematics_solver_timeout': 0.005,
                    'position_only_ik': True
                }
            }, f)
        print(f"Created kinematics.yaml file at {kinematics_file}")
    
    # Verify files exist
    if not os.path.exists(urdf_file):
        print(f"ERROR: URDF file not found at {urdf_file}")
    if not os.path.exists(srdf_file):
        print(f"ERROR: SRDF file not found at {srdf_file}")
    
    # Create a basic MoveIt configuration
    moveit_config = MoveItConfigsBuilder("pegasus_arm", package_name="pegasus_arm_moveit").to_moveit_configs()
    
    # Ensure only valid attributes are used
    # Removed any invalid references to `move_group_capabilities` or similar attributes
    
    # Load the robot description directly from file paths
    robot_description_content = load_file(urdf_file)
    if robot_description_content:
        moveit_config.robot_description = {"robot_description": robot_description_content}
    
    # Load the SRDF
    robot_description_semantic_content = load_file(srdf_file)
    if robot_description_semantic_content:
        moveit_config.robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    # Load kinematics.yaml
    kinematics_yaml = load_yaml(kinematics_file)
    if kinematics_yaml:
        moveit_config.robot_description_kinematics = kinematics_yaml
    
    # Load joint limits
    if os.path.exists(joint_limits_file):
        joint_limits_yaml = load_yaml(joint_limits_file)
        if joint_limits_yaml:
            moveit_config.robot_description_planning = joint_limits_yaml
    
    # Add controller configuration
    moveit_config.trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    
    # Get joint names from the SRDF file to use in controller configuration
    moveit_config.moveit_controllers = {
        "controller_names": ["manipulator_controller"],
        "manipulator_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        }
    }
    
    # Add planning pipeline parameters
    moveit_config.planning_pipelines = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization 
                                   default_planner_request_adapters/ResolveConstraintFrames 
                                   default_planner_request_adapters/FixWorkspaceBounds 
                                   default_planner_request_adapters/FixStartStateBounds 
                                   default_planner_request_adapters/FixStartStateCollision 
                                   default_planner_request_adapters/FixStartStatePathConstraints""",
        }
    }
    
    # Generate the MoveIt launch description
    return generate_move_group_launch(moveit_config)