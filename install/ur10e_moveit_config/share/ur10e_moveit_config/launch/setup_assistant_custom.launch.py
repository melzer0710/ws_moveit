import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # 1. Pfad zu deiner XACRO-Datei definieren
    xacro_file = os.path.join(
        get_package_share_directory('ur10e_moveit_config'), # <-- Passe den Paketnamen an, falls nötig
        'config',
        'ur10e_robot.urdf.xacro'
    )

    # 2. Die XACRO-Datei in einen URDF-String konvertieren
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 3. Den URDF-String in den ROS-Parameter "robot_description" laden
    robot_description = {'robot_description': robot_description_raw}

    # 4. Den MoveIt Setup Assistant starten (er wird den Parameter automatisch finden)
    setup_assistant_node = Node(
        package='moveit_setup_assistant',
        executable='moveit_setup_assistant',
        output='screen',
        parameters=[robot_description] # <-- Hier übergeben wir den Parameter
    )

    return LaunchDescription([setup_assistant_node])