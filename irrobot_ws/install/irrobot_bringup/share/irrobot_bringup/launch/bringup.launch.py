from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():
    share_dir = get_package_share_directory('irrobot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'irrobot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    
    return LaunchDescription([
        # Publica os estados das juntas
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
        ),

        # Controlador diferencial
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"]
        ),
        
        # Publica a URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_urdf}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
