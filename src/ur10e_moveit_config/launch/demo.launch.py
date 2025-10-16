import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    ld = LaunchDescription()

    # --- MoveIt2-Konfiguration laden ---
    moveit_config = MoveItConfigsBuilder("ur10e_robot", package_name="ur10e_moveit_config").to_moveit_configs()

    # --- Xacro in URDF konvertieren ---
    urdf_xacro_path = os.path.join(
        get_package_share_directory("ur10e_moveit_config"),
        "config",
        "ur10e_robot.urdf.xacro"
    )
    urdf_path = "/tmp/ur10e_robot.urdf"

    convert_xacro = ExecuteProcess(
        cmd=['ros2', 'run', 'xacro', 'xacro', urdf_xacro_path, '-o', urdf_path],
        output='screen'
    )
    ld.add_action(convert_xacro)

    # --- Gazebo Classic starten ---
    ld.add_action(ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
             '/usr/share/gazebo-11/worlds/empty.world'],
        output='screen'
    ))

    # --- Roboter spawnen (nach kurzer Verzögerung, damit Welt geladen ist) ---
    spawn_robot = TimerAction(
        period=6.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'ur10e_robot',
                '-file', urdf_path,
                '-x', '0', '-y', '0', '-z', '0.15'
            ],
            output='screen'
        )]
    )
    ld.add_action(spawn_robot)

    # --- Tisch spawnen ---
    table_file = os.path.join(
        get_package_share_directory("ur10e_moveit_config"),
        "config",
        "simple_table.urdf"
    )
    spawn_table = TimerAction(
        period=7.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'simple_table',
                '-file', table_file,
                '-x', '0', '-y', '0', '-z', '0.0'
            ],
            output='screen'
        )]
    )
    ld.add_action(spawn_table)

    joint_state_broadcaster = TimerAction(
        period=7.5, # nach Spawn, vor dem arm_controller
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        )]
    )
    ld.add_action(joint_state_broadcaster)

    # --- Controller Spawner starten ---
    arm_controller = TimerAction(
        period=9.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
            output="screen"
        )]
    )
    ld.add_action(arm_controller)

    # --- MoveIt Demo starten (RViz + Motion Planning) ---
    moveit_demo = generate_demo_launch(moveit_config)
    ld.add_action(moveit_demo)

    return ld
