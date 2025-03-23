from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_driver',
            executable='tello_communication_node',
            name='tello_communication_node',
            output='screen'
        ),
        Node(
            package='tello_driver',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
    ])

