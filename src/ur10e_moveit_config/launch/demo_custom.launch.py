import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur10e_robot", package_name="ur10e_moveit_config").to_moveit_configs()
    
    ld = LaunchDescription()
    
    # Demo-Launch ohne RViz
    demo_ld = generate_demo_launch(moveit_config)
    for entity in demo_ld.entities:
        # RViz-Node aus Demo überspringen
        if hasattr(entity, 'name') and entity.name == 'rviz2':
            continue
        ld.add_entity(entity)
    
    # Eigenen RViz-Node mit eigener Config
    rviz_config_path = os.path.join(get_package_share_directory('ur10e_moveit_config'),
                                    'config',
                                    'my_ur10e.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    ld.add_entity(rviz_node)
    
    return ld
