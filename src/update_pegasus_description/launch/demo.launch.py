from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("pegasus", package_name="update_pegasus_description").to_moveit_configs()
    
    # Get the standard demo launch
    demo_launch_description = generate_demo_launch(moveit_config)
    
    # Spawn pegasus_arm_controller ONLY (not joint_state_broadcaster which is already spawned)
    spawn_arm_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'pegasus_arm_controller'],
        output='screen'
    )
    
    # Delay spawning by 8 seconds to ensure everything is ready
    delayed_spawn = TimerAction(
        period=8.0,  # Increased delay
        actions=[spawn_arm_controller]
    )
    
    # Add to launch description
    demo_launch_description.add_action(delayed_spawn)
    
    return demo_launch_description
