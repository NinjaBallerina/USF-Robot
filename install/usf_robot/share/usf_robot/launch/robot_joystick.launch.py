from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='usf_robot',
            executable='joystick_control',
            name='joystick_control',
            output='screen'
        ),
        Node(
            package='usf_robot',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        )
    ])

