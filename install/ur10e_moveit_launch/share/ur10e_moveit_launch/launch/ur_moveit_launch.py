from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Roboter-Typ als Launch-Argument
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur10e',
        description='Universal Robot type'
    )
    
    ur_type = LaunchConfiguration('ur_type')

    # Pfad zum MoveIt2 Launchfile im UR MoveIt2 Config Repo
    pkg_path = os.path.join(
        os.getenv('ROS_WORKSPACE', os.getenv('ROS2_WS', os.path.expanduser('~/ros2_ws'))),
        'src',
        'Universal_Robots_ROS2_Description'
    )
    moveit_launch_path = os.path.join(pkg_path, 'launch', 'view_ur.launch.py')
    
    return LaunchDescription([
        ur_type_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_path),
            launch_arguments={'robot_ip': '', 'robot_type': ur_type}.items(),
        )
    ])
