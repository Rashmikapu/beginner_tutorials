

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="beginner_tutorials",
            executable="param_pub",
            name="param_pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"freq": 2.0}
            ]
        ),
        Node(
            package="beginner_tutorials",
            executable="param_sub",
            name="param_sub",
            output="screen",
            emulate_tty=True,
            
        )
    ])