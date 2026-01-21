from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Kinect2 Bridge (Mock/Simulated or Real)
    # Assuming kinect2_bridge package exists as per user request
    kinect_node = Node(
        package='kinect2_bridge',
        executable='kinect2_bridge',
        name='kinect2_bridge',
        output='screen'
    )

    # 2. HandCV Node
    handcv_node = Node(
        package='handcv',
        executable='handcv', # Assuming executable name matches package or is standard
        name='handcv',
        output='screen'
    )

    # 3. CV Pegasus Bridge Node
    bridge_node = Node(
        package='cv_pegasus_bridge',
        executable='cv_pegasus_bridge',
        name='cv_pegasus_bridge',
        output='screen'
    )

    # 4. Pegasus Robot Driver/Description (MoveIt + Controllers)
    # Using tester.launch.py as it seems to launch everything (controllers, moveit, rviz)
    pegasus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('update_pegasus_description'),
                'launch',
                'tester.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        kinect_node,
        handcv_node,
        bridge_node,
        pegasus_launch
    ])
