

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    record_flag = LaunchConfiguration('record_flag')
    return LaunchDescription([

        DeclareLaunchArgument(
            'record_flag',
            default_value='False'
        ),
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
            
        ),
        ExecuteProcess(
        condition=IfCondition(record_flag),
        cmd=[
            'ros2', 'bag', 'record', '-o tutorial_bag', '-a'
        ],
        shell=True
        )
    ])