from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    xacro_file = os.path.join(
        os.path.expanduser('~'),
        'irrobot_ws/src/irrobot_description/urdf/irrobot.urdf.xacro'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['xacro', xacro_file, '-o', '/tmp/irrobot.urdf'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/tmp/irrobot.urdf').read()}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
