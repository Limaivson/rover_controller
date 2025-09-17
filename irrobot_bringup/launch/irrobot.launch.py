from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    irrobot_description_share = get_package_share_directory('irrobot_description')
    irrobot_bringup_share = get_package_share_directory('irrobot_bringup')

    urdf_xacro_path = os.path.join(irrobot_description_share, 'urdf', 'irrobot.urdf.xacro')
    rviz_config_path = os.path.join(irrobot_description_share, 'config', 'display.rviz')
    controllers_yaml = os.path.join(irrobot_bringup_share, 'config', 'irrobot_controllers.yaml')

    robot_description = xacro.process_file(urdf_xacro_path).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_yaml],
            remappings=[
                ('/controller_manager/robot_description', '/robot_description')
            ]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster']
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])