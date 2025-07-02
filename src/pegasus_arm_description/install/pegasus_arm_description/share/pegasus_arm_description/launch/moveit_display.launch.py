import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, LogInfo, ExecuteProcess, Shutdown
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    """Load a YAML file from a package share directory."""
    try:
        file_path = os.path.join(get_package_share_directory(package_name), file_path)
        with open(file_path, 'r') as file:
            return yaml.safe_load(file) or {}
    except Exception as e:
        print(f"Failed to load YAML file {file_path}: {e}")
        return {}

def generate_launch_description():
    # Validate package existence
    try:
        get_package_share_directory("pegasus_arm_description")
    except Exception as e:
        return LaunchDescription([
            LogInfo(msg=f"Error: Package 'pegasus_arm_description' not found: {e}"),
            Shutdown()
        ])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare path to the Xacro file - CHANGED TO UPPERCASE URDF DIRECTORY
    model = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(
            get_package_share_directory("pegasus_arm_description"),
            "URDF",  # Changed from "urdf" to "URDF"
            "pegasus_arm.urdf.xacro"
        ),
        description='Path to robot Xacro file'
    )

    # Paths to configuration files
    try:
        pkg_share = get_package_share_directory("pegasus_arm_description")
    except Exception as e:
        return LaunchDescription([
            LogInfo(msg=f"Error: Failed to get package share directory: {e}"),
            Shutdown()
        ])

    srdf_path = os.path.join(pkg_share, "srdf", "pegasus_arm.srdf")
    rviz_config_file = os.path.join(pkg_share, "rviz", "moveit.rviz")
    ros2_controllers_yaml_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    controllers_yaml = load_yaml("pegasus_arm_description", "config/controllers.yaml")
    ompl_planning_yaml = load_yaml("pegasus_arm_description", "config/ompl_planning.yaml")

    # Default OMPL planning config if file is missing
    if not ompl_planning_yaml:
        ompl_planning_yaml = {
            "move_group": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": ["default_planner_request_adapters/AddTimeParameterization",
                                    "default_planner_request_adapters/FixWorkspaceBounds",
                                    "default_planner_request_adapters/FixStartStateBounds",
                                    "default_planner_request_adapters/FixStartStateCollision",
                                    "default_planner_request_adapters/FixStartStatePathConstraints"],
                "start_state_max_bounds_error": 0.1
            },
            "planner_configs": {
                "RRTConnect": {
                    "type": "geometric::RRTConnect",
                    "range": 0.1,
                    "goal_bias": 0.05
                }
            },
            "pegasus_arm": {
                "planner_configs": ["RRTConnect"],
                "projection_evaluator": "joints(joint1, joint2)",
                "longest_valid_segment_fraction": 0.05
            },
            "planning_plugins": ["ompl_interface/OMPLPlanner"]
        }
        print("Using default OMPL planning configuration.")

    # Validate configuration files
    if not os.path.exists(srdf_path):
        return LaunchDescription([
            LogInfo(msg=f"Error: SRDF file not found at {srdf_path}"),
            Shutdown()
        ])
    if not os.path.exists(ros2_controllers_yaml_path):
        return LaunchDescription([
            LogInfo(msg=f"Error: Controller config not found at {ros2_controllers_yaml_path}"),
            Shutdown()
        ])
    if not controllers_yaml:
        return LaunchDescription([
            LogInfo(msg="Error: controllers.yaml is empty or invalid"),
            Shutdown()
        ])
    if not os.path.exists(rviz_config_file):
        return LaunchDescription([
            LogInfo(msg=f"Error: RViz config file not found at {rviz_config_file}"),
            Shutdown()
        ])

    # Log file paths for debugging
    print(f"SRDF Path: {srdf_path}")
    print(f"Controllers YAML Path: {ros2_controllers_yaml_path}")
    print(f"RViz Config Path: {rviz_config_file}")

    # Robot description
    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Semantic description (SRDF)
    try:
        with open(srdf_path, 'r') as srdf_file:
            robot_description_semantic_content = srdf_file.read()
            print(f"Successfully loaded SRDF content")
    except Exception as e:
        return LaunchDescription([
            LogInfo(msg=f"Error: Could not load SRDF file {srdf_path}: {e}"),
            Shutdown()
        ])

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # Robot State Publisher
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
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'base_link']
    )

    # Controller Manager
    ros2_controllers_yaml_path = r"~/ros2-ws/src/pegasus_arm_description/config/ros2_controllers.yaml"
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
            ros2_controllers_yaml_path
            
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '30'],
        output='screen',
        parameters=[{'type': 'joint_state_broadcaster/JointStateBroadcaster'}]
    )

    # Arm Controller Spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pegasus_arm_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '30'],
        output='screen',
        parameters=[{'type': 'joint_trajectory_controller/JointTrajectoryController'}]
    )

    # MoveIt MoveGroup
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_yaml,
            {'use_sim_time': use_sim_time},
            {
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_simple_controller_manager': controllers_yaml,
                'publish_planning_scene': True,
                'planning_scene_monitor.publish_planning_scene': True,
                'planning_scene_monitor.publish_geometry_updates': True,
                'planning_scene_monitor.publish_state_updates': True,
                'planning_scene_monitor.publish_transforms_updates': True,
                'planning_attempts': 3,
                'max_velocity_scaling_factor': 1.0,
                'max_acceleration_scaling_factor': 1.0,
                'moveit_manage_controllers': True,
                'trajectory_execution.allowed_execution_duration_scaling': 2.0,
                'trajectory_execution.allowed_goal_duration_margin': 1.0,
                'trajectory_execution.allowed_start_tolerance': 0.01,
                'planning_scene_monitor.request_planning_scene_state': True,
                'allow_trajectory_execution': True,
                'robot_model_name': 'pegasus_arm'
            }
        ],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Check topic publication
    check_topics = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen'
    )

    # Event handlers
    rsp_ready_event = RegisterEventHandler(
        OnProcessStart(
            target_action=rsp_node,
            on_start=[
                LogInfo(msg="Robot State Publisher started, waiting for TF tree..."),
                TimerAction(
                    period=5.0,
                    actions=[
                        LogInfo(msg="Starting Controller Manager..."),
                        controller_manager_node
                    ]
                )
            ]
        )
    )

    controller_manager_ready_event = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                LogInfo(msg="Controller Manager started, waiting for services..."),
                TimerAction(
                    period=15.0,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )

    controller_manager_failure_event = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[
                LogInfo(msg="Controller Manager failed! Check ros2_controllers.yaml, URDF, or mock hardware interface."),
                Shutdown()
            ]
        )
    )

    joint_state_ready_event = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg="Joint State Broadcaster started, spawning Pegasus Arm Controller..."),
                arm_controller_spawner
            ]
        )
    )

    arm_controller_ready_event = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                LogInfo(msg="Pegasus Arm Controller spawned, starting MoveIt MoveGroup..."),
                TimerAction(
                    period=5.0,
                    actions=[move_group_node]
                )
            ]
        )
    )

    move_group_ready_event = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[
                LogInfo(msg="MoveIt MoveGroup started, launching RViz and checking topics..."),
                TimerAction(
                    period=5.0,
                    actions=[rviz_node, check_topics]
                )
            ]
        )
    )

    move_group_failure_event = RegisterEventHandler(
        OnProcessExit(
            target_action=move_group_node,
            on_exit=[
                LogInfo(msg="MoveIt MoveGroup failed! Check MoveIt configuration or logs."),
                Shutdown()
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        model,
        LogInfo(msg="Starting Pegasus Arm MoveIt Launch..."),
        rsp_node,
        static_tf_publisher,
        rsp_ready_event,
        controller_manager_ready_event,
        controller_manager_failure_event,
        joint_state_ready_event,
        arm_controller_ready_event,
        move_group_ready_event,
        move_group_failure_event
    ])