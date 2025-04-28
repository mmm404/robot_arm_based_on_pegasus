from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "pegasus_arm",
        package_name="moveit_srdf"
    ).to_moveit_configs()
    print(moveit_config)  # Debugging: Print the moveit_config object
    return generate_setup_assistant_launch(moveit_config)