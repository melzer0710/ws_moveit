import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("ur10e_robot", package_name="ur10e_moveit_config").to_moveit_configs()

    # 1. Pfade zu den Konfigurationsdateien definieren (wie im Tutorial)
    controllers_config_path = PathJoinSubstitution(
        [FindPackageShare("ur10e_moveit_config"), "config", "ros2_controllers.yaml"]
    )
    initial_positions_path = PathJoinSubstitution(
        [FindPackageShare("ur10e_moveit_config"), "config", "initial_positions.yaml"]
    )
    xacro_file_path = PathJoinSubstitution(
        [FindPackageShare("ur10e_moveit_config"), "config", "ur10e_robot.urdf.xacro"]
    )

    # 2. 'robot_description' erstellen und die Pfade als Argumente an XACRO übergeben
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            xacro_file_path, " ",
            "controllers_file:=", controllers_config_path, " ",
            "initial_positions_file:=", initial_positions_path,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "ur10e_robot"],
        output="screen",
    )

    # 3. Der externe 'controller_manager_node' WURDE HIER ENTFERNT

    # 4. Spawner starten, NACHDEM der Roboter gespawnt ist (Timing ist wichtig)
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                ),
            ],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("ur10e_moveit_config"), "config", "moveit.rviz"])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            gazebo,
            spawn_entity,
            load_controllers, # Der externe 'controller_manager_node' ist hier entfernt
            rviz_node,
            move_group_node,
        ]
    )